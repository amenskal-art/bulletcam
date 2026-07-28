#include "dense/patchmatch.h"

#include <cstdio>

#include "core/parallel.h"

namespace bc {
namespace {

constexpr float kInvalidCost = 3.f;
constexpr int kMaxPatchSamples = 121;
constexpr float kMinValidSampleRatio = 0.6f;

// Pre-transformed source view: everything that does not depend on the plane
// hypothesis is folded into A (= K_src * R_rel * K_ref^-1) and b (= K_src * t_rel).
struct SourceContext {
    const ImageU8* gray = nullptr;
    Mat3 A;
    Vec3 b;
};

struct RefPatch {
    int count = 0;
    float px[kMaxPatchSamples];
    float py[kMaxPatchSamples];
    float value[kMaxPatchSamples];
    float rayX[kMaxPatchSamples];  // K_ref^-1 * p, x component
    float rayY[kMaxPatchSamples];
    Vec3 rayCenter;
};

struct Plane {
    float depth = 0.f;
    Vec3 normal;
};

Vec3 randomHemisphereNormal(Rng& rng, const Vec3& rayUnit, float maxAngleRad) {
    // Build an orthonormal basis around the inward axis (-ray).
    const Vec3 axis = -rayUnit;
    Vec3 helper = std::fabs(axis.x) < 0.9f ? Vec3{1.f, 0.f, 0.f} : Vec3{0.f, 1.f, 0.f};
    const Vec3 u = axis.cross(helper).normalized();
    const Vec3 v = axis.cross(u);
    const float cosMax = std::cos(maxAngleRad);
    const float cosTheta = rng.uniform(cosMax, 1.f);
    const float sinTheta = std::sqrt(std::max(0.f, 1.f - cosTheta * cosTheta));
    const float phi = rng.uniform(0.f, 6.2831853f);
    return (axis * cosTheta + u * (sinTheta * std::cos(phi)) + v * (sinTheta * std::sin(phi)))
        .normalized();
}

Vec3 perturbNormal(Rng& rng, const Vec3& normal, const Vec3& rayUnit, float maxAngleRad) {
    Vec3 helper = std::fabs(normal.x) < 0.9f ? Vec3{1.f, 0.f, 0.f} : Vec3{0.f, 1.f, 0.f};
    const Vec3 u = normal.cross(helper).normalized();
    const Vec3 v = normal.cross(u);
    const float theta = rng.uniform(0.f, maxAngleRad);
    const float phi = rng.uniform(0.f, 6.2831853f);
    Vec3 n = (normal * std::cos(theta) +
              (u * std::cos(phi) + v * std::sin(phi)) * std::sin(theta))
                 .normalized();
    // Keep the plane facing the camera; a back-facing plane has no valid warp.
    if (n.dot(rayUnit) > -0.05f) {
        n = (n - rayUnit * (n.dot(rayUnit) + 0.2f)).normalized();
        if (n.dot(rayUnit) > -0.02f) return normal;
    }
    return n;
}

class PatchMatcher {
public:
    PatchMatcher(const FrameStore& store, const SparseMap& sparse, int refIndex,
                 const std::vector<int>& sourceViews)
        : options_(store.options()),
          ref_(store.frames()[static_cast<size_t>(refIndex)]),
          refGray_(store.frames()[static_cast<size_t>(refIndex)].gray),
          sparse_(sparse),
          width_(store.frames()[static_cast<size_t>(refIndex)].gray.width),
          height_(store.frames()[static_cast<size_t>(refIndex)].gray.height) {
        const View& refView = ref_.stereoView;
        const Mat3 kInv = refView.intrinsics.inverseMatrix();
        for (const int s : sourceViews) {
            const Frame& src = store.frames()[static_cast<size_t>(s)];
            Mat3 Rrel;
            Vec3 trel;
            relativePose(refView, src.stereoView, Rrel, trel);
            SourceContext ctx;
            ctx.gray = &src.gray;
            const Mat3 kSrc = src.stereoView.intrinsics.matrix();
            ctx.A = kSrc * Rrel * kInv;
            ctx.b = kSrc * trel;
            sources_.push_back(ctx);
        }
        radius_ = std::min(5, std::max(1, options_.patchRadius));
        step_ = std::max(1, options_.patchStep);
    }

    void run(DepthMap& out) {
        out.resize(width_, height_);
        if (sources_.empty()) return;
        depth_ = &out.depth;
        normal_ = &out.normal;
        cost_ = &out.cost;
        initialise();
        for (int iter = 0; iter < options_.patchMatchIterations; ++iter) {
            const float shrink = std::pow(0.5f, static_cast<float>(iter));
            sweepRows(true, shrink);
            sweepColumns(true, shrink);
            sweepRows(false, shrink);
            sweepColumns(false, shrink);
        }
        // Drop hypotheses the photometric term never agreed on.
        const float maxCost = 1.f - options_.minNccScore;
        for (size_t i = 0; i < out.depth.size(); ++i) {
            if (out.cost[i] > maxCost) out.depth[i] = 0.f;
        }
    }

private:
    void buildRefPatch(int x, int y, RefPatch& patch) const {
        const Pinhole& k = ref_.stereoView.intrinsics;
        patch.count = 0;
        for (int dy = -radius_; dy <= radius_; dy += step_) {
            for (int dx = -radius_; dx <= radius_; dx += step_) {
                const int sx = std::min(width_ - 1, std::max(0, x + dx));
                const int sy = std::min(height_ - 1, std::max(0, y + dy));
                const int i = patch.count++;
                patch.px[i] = static_cast<float>(sx);
                patch.py[i] = static_cast<float>(sy);
                patch.value[i] = static_cast<float>(refGray_.at(sx, sy));
                patch.rayX[i] = (static_cast<float>(sx) - k.cx) / k.fx;
                patch.rayY[i] = (static_cast<float>(sy) - k.cy) / k.fy;
            }
        }
        patch.rayCenter = k.ray(static_cast<float>(x), static_cast<float>(y));
    }

    // Zero-mean normalised cross correlation of the plane-warped window.
    float sourceCost(const SourceContext& src, const RefPatch& patch, const Plane& plane) const {
        const float nDotRayC = plane.normal.dot(patch.rayCenter);
        if (nDotRayC > -1e-3f) return kInvalidCost;  // plane edge-on or back-facing
        const float invDist = 1.f / (plane.depth * nDotRayC);

        const float maxX = static_cast<float>(src.gray->width - 1);
        const float maxY = static_cast<float>(src.gray->height - 1);

        float sumR = 0.f, sumR2 = 0.f, sumS = 0.f, sumS2 = 0.f, sumRS = 0.f;
        int valid = 0;
        for (int i = 0; i < patch.count; ++i) {
            const float px = patch.px[i];
            const float py = patch.py[i];
            const float scale =
                (plane.normal.x * patch.rayX[i] + plane.normal.y * patch.rayY[i] +
                 plane.normal.z) *
                invDist;
            const float qx = src.A(0, 0) * px + src.A(0, 1) * py + src.A(0, 2) + src.b.x * scale;
            const float qy = src.A(1, 0) * px + src.A(1, 1) * py + src.A(1, 2) + src.b.y * scale;
            const float qz = src.A(2, 0) * px + src.A(2, 1) * py + src.A(2, 2) + src.b.z * scale;
            if (qz <= 1e-6f) continue;
            const float sx = qx / qz;
            const float sy = qy / qz;
            if (sx < 0.f || sy < 0.f || sx > maxX || sy > maxY) continue;
            const float s = src.gray->sampleBilinear(sx, sy);
            const float r = patch.value[i];
            sumR += r;
            sumR2 += r * r;
            sumS += s;
            sumS2 += s * s;
            sumRS += r * s;
            ++valid;
        }
        if (valid < 4 ||
            static_cast<float>(valid) < kMinValidSampleRatio * static_cast<float>(patch.count))
            return kInvalidCost;

        const float n = static_cast<float>(valid);
        const float varR = n * sumR2 - sumR * sumR;
        const float varS = n * sumS2 - sumS * sumS;
        if (varR < 1e-3f || varS < 1e-3f) return kInvalidCost;  // textureless window
        const float ncc = (n * sumRS - sumR * sumS) / std::sqrt(varR * varS);
        return 1.f - clampf(ncc, -1.f, 1.f);
    }

    // Aggregates over sources by averaging the best half - robust to the
    // occlusions that are unavoidable when orbiting a solid object.
    float planeCost(const RefPatch& patch, const Plane& plane) const {
        if (plane.depth <= 1e-4f || !plane.normal.isFinite()) return kInvalidCost;
        float costs[32];
        int count = 0;
        for (const SourceContext& src : sources_) {
            const float c = sourceCost(src, patch, plane);
            if (c < kInvalidCost) costs[count++] = c;
            if (count == 32) break;
        }
        if (count == 0) return kInvalidCost;
        std::sort(costs, costs + count);
        const int best = std::max(1, count / 2);
        float sum = 0.f;
        for (int i = 0; i < best; ++i) sum += costs[i];
        return sum / static_cast<float>(best);
    }

    // Projects the sparse cloud into this view to seed the depth search. Without
    // a prior, PatchMatch needs many more iterations to lock on.
    void initialise() {
        const View& view = ref_.stereoView;
        const int stride = 8;
        const int gw = (width_ + stride - 1) / stride;
        const int gh = (height_ + stride - 1) / stride;
        std::vector<float> seed(static_cast<size_t>(gw) * gh, 0.f);
        std::vector<int> seedCount(static_cast<size_t>(gw) * gh, 0);

        for (const SparsePoint& p : sparse_.points) {
            const Vec3 cam = view.worldToCamera(p.position);
            if (cam.z <= 1e-3f) continue;
            const Vec2 px = view.intrinsics.project(cam);
            const int gx = static_cast<int>(px.x) / stride;
            const int gy = static_cast<int>(px.y) / stride;
            if (gx < 0 || gy < 0 || gx >= gw || gy >= gh) continue;
            const size_t idx = static_cast<size_t>(gy) * gw + gx;
            seed[idx] += cam.z;
            seedCount[idx] += 1;
        }
        for (size_t i = 0; i < seed.size(); ++i)
            if (seedCount[i] > 0) seed[i] /= static_cast<float>(seedCount[i]);

        // Dilate the seed grid so pixels between sparse observations still start
        // close to the right depth.
        for (int pass = 0; pass < 6; ++pass) {
            std::vector<float> next = seed;
            for (int y = 0; y < gh; ++y) {
                for (int x = 0; x < gw; ++x) {
                    const size_t idx = static_cast<size_t>(y) * gw + x;
                    if (seed[idx] > 0.f) continue;
                    float sum = 0.f;
                    int n = 0;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            const int nx = x + dx, ny = y + dy;
                            if (nx < 0 || ny < 0 || nx >= gw || ny >= gh) continue;
                            const float v = seed[static_cast<size_t>(ny) * gw + nx];
                            if (v > 0.f) { sum += v; ++n; }
                        }
                    }
                    if (n > 0) next[idx] = sum / static_cast<float>(n);
                }
            }
            seed.swap(next);
        }

        const float dMin = std::max(1e-3f, ref_.depthMin);
        const float dMax = std::max(dMin * 1.2f, ref_.depthMax);
        const float logMin = std::log(dMin);
        const float logMax = std::log(dMax);

        parallelBlocks(height_, [&](int y0, int y1) {
            Rng rng(static_cast<uint32_t>(0x1234567u + y0 * 7919 + ref_.id * 104729));
            RefPatch patch;
            for (int y = y0; y < y1; ++y) {
                for (int x = 0; x < width_; ++x) {
                    const size_t idx = static_cast<size_t>(y) * width_ + x;
                    const float prior = seed[static_cast<size_t>(y / stride) * gw + (x / stride)];
                    float d;
                    if (prior > 0.f) {
                        d = clampf(prior * rng.uniform(0.82f, 1.22f), dMin, dMax);
                    } else {
                        d = std::exp(rng.uniform(logMin, logMax));
                    }
                    const Vec3 rayUnit =
                        ref_.stereoView.intrinsics.ray(static_cast<float>(x), static_cast<float>(y))
                            .normalized();
                    Plane plane;
                    plane.depth = d;
                    plane.normal = randomHemisphereNormal(rng, rayUnit, 1.15f);
                    buildRefPatch(x, y, patch);
                    store(idx, plane, planeCost(patch, plane));
                }
            }
        });
    }

    void store(size_t idx, const Plane& plane, float cost) {
        (*depth_)[idx] = plane.depth;
        (*normal_)[idx * 3 + 0] = plane.normal.x;
        (*normal_)[idx * 3 + 1] = plane.normal.y;
        (*normal_)[idx * 3 + 2] = plane.normal.z;
        (*cost_)[idx] = cost;
    }

    Plane load(size_t idx) const {
        Plane p;
        p.depth = (*depth_)[idx];
        p.normal = {(*normal_)[idx * 3 + 0], (*normal_)[idx * 3 + 1], (*normal_)[idx * 3 + 2]};
        return p;
    }

    // Re-evaluates a neighbour's plane at this pixel's ray instead of copying its
    // depth, which is what makes PatchMatch handle slanted surfaces.
    Plane propagate(const Plane& neighbour, const Vec3& neighbourRay, const Vec3& ownRay) const {
        Plane p;
        p.normal = neighbour.normal;
        const float dist = neighbour.normal.dot(neighbourRay * neighbour.depth);
        const float denom = neighbour.normal.dot(ownRay);
        if (std::fabs(denom) < 1e-6f) {
            p.depth = 0.f;
            return p;
        }
        p.depth = dist / denom;
        return p;
    }

    void improve(int x, int y, int nx, int ny, float shrink) {
        const size_t idx = static_cast<size_t>(y) * width_ + x;
        const Pinhole& k = ref_.stereoView.intrinsics;
        RefPatch patch;
        buildRefPatch(x, y, patch);

        Plane best = load(idx);
        float bestCost = (*cost_)[idx];
        const Vec3 ownRay = patch.rayCenter;
        const Vec3 ownRayUnit = ownRay.normalized();

        if (nx >= 0 && ny >= 0) {
            const size_t nIdx = static_cast<size_t>(ny) * width_ + nx;
            const Plane neighbour = load(nIdx);
            if (neighbour.depth > 1e-4f) {
                const Vec3 nRay = k.ray(static_cast<float>(nx), static_cast<float>(ny));
                const Plane candidate = propagate(neighbour, nRay, ownRay);
                if (candidate.depth > ref_.depthMin * 0.5f &&
                    candidate.depth < ref_.depthMax * 2.f) {
                    const float c = planeCost(patch, candidate);
                    if (c < bestCost) {
                        bestCost = c;
                        best = candidate;
                    }
                }
            }
        }

        Rng rng(static_cast<uint32_t>(idx * 2654435761u + rngSalt_));
        const float depthRange = std::max(1e-4f, best.depth * 0.22f * shrink);
        const float angleRange = 0.9f * shrink;
        for (int attempt = 0; attempt < 2; ++attempt) {
            Plane candidate;
            candidate.depth = best.depth > 1e-4f
                                  ? best.depth + rng.uniform(-depthRange, depthRange)
                                  : rng.uniform(ref_.depthMin, ref_.depthMax);
            candidate.depth = clampf(candidate.depth, ref_.depthMin * 0.5f, ref_.depthMax * 2.f);
            candidate.normal = best.normal.squaredNorm() > 0.5f
                                   ? perturbNormal(rng, best.normal, ownRayUnit, angleRange)
                                   : randomHemisphereNormal(rng, ownRayUnit, 1.15f);
            const float c = planeCost(patch, candidate);
            if (c < bestCost) {
                bestCost = c;
                best = candidate;
            }
        }
        store(idx, best, bestCost);
    }

    void sweepRows(bool forward, float shrink) {
        rngSalt_ += 0x9E3779B9u;
        parallelBlocks(height_, [&](int y0, int y1) {
            for (int y = y0; y < y1; ++y) {
                if (forward) {
                    for (int x = 0; x < width_; ++x) improve(x, y, x - 1, y, shrink);
                } else {
                    for (int x = width_ - 1; x >= 0; --x) improve(x, y, x + 1 < width_ ? x + 1 : -1, y, shrink);
                }
            }
        });
    }

    void sweepColumns(bool forward, float shrink) {
        rngSalt_ += 0x85EBCA6Bu;
        parallelBlocks(width_, [&](int x0, int x1) {
            for (int x = x0; x < x1; ++x) {
                if (forward) {
                    for (int y = 0; y < height_; ++y) improve(x, y, x, y - 1, shrink);
                } else {
                    for (int y = height_ - 1; y >= 0; --y)
                        improve(x, y, x, y + 1 < height_ ? y + 1 : -1, shrink);
                }
            }
        });
    }

    const ReconstructionOptions& options_;
    const Frame& ref_;
    const ImageU8& refGray_;
    const SparseMap& sparse_;
    std::vector<SourceContext> sources_;
    int width_ = 0, height_ = 0;
    int radius_ = 4, step_ = 2;
    uint32_t rngSalt_ = 0x2545F491u;
    std::vector<float>* depth_ = nullptr;
    std::vector<float>* normal_ = nullptr;
    std::vector<float>* cost_ = nullptr;
};

}  // namespace

void computeDepthMap(const FrameStore& store, const SparseMap& sparse, int refIndex,
                     const std::vector<int>& sourceViews, DepthMap& out) {
    PatchMatcher matcher(store, sparse, refIndex, sourceViews);
    matcher.run(out);
}

bool saveNormalMap(const std::string& path, const DepthMap& map) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const size_t n = map.normal.size();
    const bool ok = std::fwrite(map.normal.data(), sizeof(float), n, f) == n;
    std::fclose(f);
    return ok;
}

bool loadNormalMap(const std::string& path, int width, int height, std::vector<float>& normals) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    normals.assign(static_cast<size_t>(width) * height * 3, 0.f);
    const bool ok = std::fread(normals.data(), sizeof(float), normals.size(), f) == normals.size();
    std::fclose(f);
    return ok;
}

}  // namespace bc
