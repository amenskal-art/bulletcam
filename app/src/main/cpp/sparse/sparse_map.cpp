#include "sparse/sparse_map.h"

#include <algorithm>
#include <mutex>

#include "core/log.h"
#include "core/parallel.h"

namespace bc {
namespace {

constexpr int kMaxFeatures = 1400;
constexpr int kFastThreshold = 18;
constexpr int kNeighbourCandidates = 8;
constexpr int kMaxHamming = 64;
constexpr float kLoweRatio = 0.78f;
constexpr float kMaxEpipolarPx = 2.5f;
constexpr float kMaxReprojectionPx = 2.5f;

// Uniform bucket grid over one image, used to restrict descriptor comparisons
// to the neighbourhood of an epipolar line.
struct KeypointGrid {
    int cols = 0, rows = 0, cell = 24;
    std::vector<std::vector<int>> buckets;

    KeypointGrid(const std::vector<Keypoint>& kps, int width, int height) {
        cols = std::max(1, width / cell + 1);
        rows = std::max(1, height / cell + 1);
        buckets.resize(static_cast<size_t>(cols) * rows);
        for (size_t i = 0; i < kps.size(); ++i) {
            const int cx = std::min(cols - 1, static_cast<int>(kps[i].x) / cell);
            const int cy = std::min(rows - 1, static_cast<int>(kps[i].y) / cell);
            buckets[static_cast<size_t>(cy) * cols + cx].push_back(static_cast<int>(i));
        }
    }

    // Collects every keypoint whose cell is within `tolerance` of the line
    // ax + by + c = 0 (already normalised so that a^2 + b^2 == 1).
    void queryLine(float a, float b, float c, float tolerance, std::vector<int>& out) const {
        out.clear();
        const float cellRadius = 0.7072f * static_cast<float>(cell);
        for (int cy = 0; cy < rows; ++cy) {
            const float py = (cy + 0.5f) * static_cast<float>(cell);
            for (int cx = 0; cx < cols; ++cx) {
                const auto& bucket = buckets[static_cast<size_t>(cy) * cols + cx];
                if (bucket.empty()) continue;
                const float px = (cx + 0.5f) * static_cast<float>(cell);
                if (std::fabs(a * px + b * py + c) > tolerance + cellRadius) continue;
                out.insert(out.end(), bucket.begin(), bucket.end());
            }
        }
    }
};

float percentile(std::vector<float>& values, float q) {
    if (values.empty()) return 0.f;
    const size_t idx = std::min(values.size() - 1,
                                static_cast<size_t>(q * static_cast<float>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<long>(idx), values.end());
    return values[idx];
}

}  // namespace

bool triangulateRays(const View& a, const View& b, const Vec2& pa, const Vec2& pb,
                     float minAngleDeg, float maxAngleDeg, Vec3& outPoint, float& outAngleDeg) {
    const Vec3 da = a.worldRay(pa.x, pa.y);
    const Vec3 db = b.worldRay(pb.x, pb.y);

    const float cosAngle = clampf(da.dot(db), -1.f, 1.f);
    outAngleDeg = std::acos(cosAngle) * 57.2957795f;
    if (outAngleDeg < minAngleDeg || outAngleDeg > maxAngleDeg) return false;

    // Closest points on the two rays (classic midpoint triangulation).
    const Vec3 w = a.center - b.center;
    const float bDot = da.dot(db);
    const float d = da.dot(w);
    const float e = db.dot(w);
    const float denom = 1.f - bDot * bDot;
    if (std::fabs(denom) < 1e-8f) return false;
    const float sA = (bDot * e - d) / denom;
    const float sB = (e - bDot * d) / denom;
    if (sA <= 0.f || sB <= 0.f) return false;  // behind one of the cameras

    outPoint = ((a.center + da * sA) + (b.center + db * sB)) * 0.5f;
    if (!outPoint.isFinite()) return false;

    const Vec3 ca = a.worldToCamera(outPoint);
    const Vec3 cb = b.worldToCamera(outPoint);
    if (ca.z <= 1e-4f || cb.z <= 1e-4f) return false;
    const float errA = (a.intrinsics.project(ca) - pa).norm();
    const float errB = (b.intrinsics.project(cb) - pb).norm();
    if (errA > kMaxReprojectionPx || errB > kMaxReprojectionPx) return false;
    return true;
}

SparseMap buildSparseMap(FrameStore& store, Progress& progress) {
    SparseMap map;
    auto& frames = store.frames();
    const int n = static_cast<int>(frames.size());
    map.viewCount = n;
    map.sharedPoints.assign(static_cast<size_t>(n) * n, 0);
    if (n < 2) return map;

    // --- Feature extraction -------------------------------------------------
    std::vector<FeatureSet> features(static_cast<size_t>(n));
    std::atomic<int> done{0};
    parallelFor(n, [&](int i) {
        features[static_cast<size_t>(i)] =
            detectAndDescribe(frames[static_cast<size_t>(i)].gray, kMaxFeatures, kFastThreshold);
        done.fetch_add(1, std::memory_order_relaxed);
    });
    progress.report(0.35f);
    if (progress.cancelled()) return map;

    // --- Candidate pairs from the known poses -------------------------------
    std::vector<std::pair<int, int>> pairs;
    for (int i = 0; i < n; ++i) {
        std::vector<std::pair<float, int>> ranked;
        const View& vi = frames[static_cast<size_t>(i)].stereoView;
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            const View& vj = frames[static_cast<size_t>(j)].stereoView;
            const float cosView = clampf(vi.forward().dot(vj.forward()), -1.f, 1.f);
            const float angle = std::acos(cosView) * 57.2957795f;
            if (angle > 65.f) continue;
            ranked.emplace_back((vi.center - vj.center).norm(), j);
        }
        std::sort(ranked.begin(), ranked.end());
        const int take = std::min<int>(kNeighbourCandidates, static_cast<int>(ranked.size()));
        for (int k = 0; k < take; ++k) {
            const int j = ranked[static_cast<size_t>(k)].second;
            if (i < j) pairs.emplace_back(i, j);
            else pairs.emplace_back(j, i);
        }
    }
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());

    // --- Epipolar-guided matching and ray triangulation ---------------------
    std::vector<KeypointGrid> grids;
    grids.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        grids.emplace_back(features[static_cast<size_t>(i)].keypoints,
                           frames[static_cast<size_t>(i)].gray.width,
                           frames[static_cast<size_t>(i)].gray.height);
    }

    std::mutex resultMutex;
    const float minAngle = store.options().minTriangulationAngleDeg;
    const float maxAngle = store.options().maxTriangulationAngleDeg;

    parallelFor(static_cast<int>(pairs.size()), [&](int p) {
        const int i = pairs[static_cast<size_t>(p)].first;
        const int j = pairs[static_cast<size_t>(p)].second;
        const View& vi = frames[static_cast<size_t>(i)].stereoView;
        const View& vj = frames[static_cast<size_t>(j)].stereoView;
        const FeatureSet& fi = features[static_cast<size_t>(i)];
        const FeatureSet& fj = features[static_cast<size_t>(j)];
        if (fi.size() < 8 || fj.size() < 8) return;

        const Mat3 F = fundamentalMatrix(vi, vj);
        std::vector<int> candidates;
        std::vector<SparsePoint> localPoints;
        int sharedCount = 0;

        for (size_t a = 0; a < fi.size(); ++a) {
            const Keypoint& ka = fi.keypoints[a];
            const Vec3 line = F * Vec3{ka.x, ka.y, 1.f};
            const float norm = std::sqrt(line.x * line.x + line.y * line.y);
            if (norm < 1e-9f) continue;
            grids[static_cast<size_t>(j)].queryLine(line.x / norm, line.y / norm, line.z / norm,
                                                    kMaxEpipolarPx, candidates);
            if (candidates.size() < 2) continue;

            int best = -1, bestDist = 1 << 30, secondDist = 1 << 30;
            for (int cand : candidates) {
                const Keypoint& kb = fj.keypoints[static_cast<size_t>(cand)];
                if (std::fabs(line.x * kb.x + line.y * kb.y + line.z) / norm > kMaxEpipolarPx)
                    continue;
                const int dist = hammingDistance(fi.descriptors[a],
                                                 fj.descriptors[static_cast<size_t>(cand)]);
                if (dist < bestDist) {
                    secondDist = bestDist;
                    bestDist = dist;
                    best = cand;
                } else if (dist < secondDist) {
                    secondDist = dist;
                }
            }
            if (best < 0 || bestDist > kMaxHamming) continue;
            if (static_cast<float>(bestDist) > kLoweRatio * static_cast<float>(secondDist)) continue;

            const Keypoint& kb = fj.keypoints[static_cast<size_t>(best)];
            Vec3 point;
            float angle = 0.f;
            if (!triangulateRays(vi, vj, {ka.x, ka.y}, {kb.x, kb.y}, minAngle, maxAngle, point,
                                 angle))
                continue;

            SparsePoint sp;
            sp.position = point;
            sp.viewA = i;
            sp.viewB = j;
            localPoints.push_back(sp);
            ++sharedCount;
        }

        if (localPoints.empty()) return;
        std::lock_guard<std::mutex> lock(resultMutex);
        map.points.insert(map.points.end(), localPoints.begin(), localPoints.end());
        map.sharedPoints[static_cast<size_t>(i) * n + j] = sharedCount;
        map.sharedPoints[static_cast<size_t>(j) * n + i] = sharedCount;
    });

    progress.report(0.8f);
    BC_LOGI("Sparse stage: %zu points from %zu pairs", map.points.size(), pairs.size());
    if (map.points.empty()) return map;

    // --- Robust bounds: trim the tails so a few stray points cannot blow up
    // the reconstruction volume.
    {
        std::vector<float> xs, ys, zs;
        xs.reserve(map.points.size());
        ys.reserve(map.points.size());
        zs.reserve(map.points.size());
        for (const auto& p : map.points) {
            xs.push_back(p.position.x);
            ys.push_back(p.position.y);
            zs.push_back(p.position.z);
        }
        std::vector<float> tmp;
        auto trimmed = [&](std::vector<float>& v, float q) {
            tmp = v;
            return percentile(tmp, q);
        };
        map.bounds.lo = {trimmed(xs, 0.02f), trimmed(ys, 0.02f), trimmed(zs, 0.02f)};
        map.bounds.hi = {trimmed(xs, 0.98f), trimmed(ys, 0.98f), trimmed(zs, 0.98f)};
        map.centroid = map.bounds.center();
    }

    // --- Per-view depth priors ---------------------------------------------
    std::vector<std::vector<float>> depths(static_cast<size_t>(n));
    for (const auto& p : map.points) {
        for (const int view : {p.viewA, p.viewB}) {
            const float z = frames[static_cast<size_t>(view)].stereoView.worldToCamera(p.position).z;
            if (z > 1e-3f) depths[static_cast<size_t>(view)].push_back(z);
        }
    }
    std::vector<float> globalDepths;
    for (const auto& d : depths) globalDepths.insert(globalDepths.end(), d.begin(), d.end());
    const float globalLo = percentile(globalDepths, 0.02f) * 0.7f;
    const float globalHi = percentile(globalDepths, 0.98f) * 1.3f;

    for (int i = 0; i < n; ++i) {
        Frame& f = frames[static_cast<size_t>(i)];
        auto& d = depths[static_cast<size_t>(i)];
        if (d.size() < 24) {
            f.depthMin = std::max(0.05f, globalLo);
            f.depthMax = std::max(f.depthMin * 1.5f, globalHi);
        } else {
            f.depthMin = std::max(0.05f, percentile(d, 0.03f) * 0.75f);
            f.depthMax = std::max(f.depthMin * 1.5f, percentile(d, 0.97f) * 1.35f);
        }
    }
    progress.report(1.f);
    return map;
}

}  // namespace bc
