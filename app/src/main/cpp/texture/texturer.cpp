#include "texture/texturer.h"

#include <algorithm>

#include "core/log.h"
#include "core/parallel.h"

namespace bc {
namespace {

struct Gbuffer {
    int resolution = 0;
    std::vector<Vec3> position;
    std::vector<Vec3> normal;
    std::vector<uint8_t> covered;
};

// Rasterises the UV triangles to recover the surface point behind each texel.
Gbuffer rasteriseAtlas(const Mesh& mesh, int resolution) {
    Gbuffer g;
    g.resolution = resolution;
    const size_t texels = static_cast<size_t>(resolution) * resolution;
    g.position.assign(texels, Vec3{0.f, 0.f, 0.f});
    g.normal.assign(texels, Vec3{0.f, 0.f, 0.f});
    g.covered.assign(texels, 0);

    const float res = static_cast<float>(resolution);
    const bool hasNormals = mesh.normals.size() == mesh.positions.size();

    parallelBlocks(static_cast<int>(mesh.triangleCount()), [&](int begin, int end) {
        for (int t = begin; t < end; ++t) {
            const uint32_t ia = mesh.indices[static_cast<size_t>(t) * 3 + 0];
            const uint32_t ib = mesh.indices[static_cast<size_t>(t) * 3 + 1];
            const uint32_t ic = mesh.indices[static_cast<size_t>(t) * 3 + 2];
            const Vec2 a{mesh.uvs[ia].x * res, mesh.uvs[ia].y * res};
            const Vec2 b{mesh.uvs[ib].x * res, mesh.uvs[ib].y * res};
            const Vec2 c{mesh.uvs[ic].x * res, mesh.uvs[ic].y * res};

            const float area = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
            if (std::fabs(area) < 1e-9f) continue;
            const float invArea = 1.f / area;

            // One texel of conservative dilation avoids pinholes on thin slivers.
            int minX = std::max(0, static_cast<int>(std::floor(std::min(a.x, std::min(b.x, c.x)))) - 1);
            int maxX = std::min(resolution - 1, static_cast<int>(std::ceil(std::max(a.x, std::max(b.x, c.x)))) + 1);
            int minY = std::max(0, static_cast<int>(std::floor(std::min(a.y, std::min(b.y, c.y)))) - 1);
            int maxY = std::min(resolution - 1, static_cast<int>(std::ceil(std::max(a.y, std::max(b.y, c.y)))) + 1);

            for (int y = minY; y <= maxY; ++y) {
                for (int x = minX; x <= maxX; ++x) {
                    const float px = static_cast<float>(x) + 0.5f;
                    const float py = static_cast<float>(y) + 0.5f;
                    float w0 = ((b.x - px) * (c.y - py) - (b.y - py) * (c.x - px)) * invArea;
                    float w1 = ((c.x - px) * (a.y - py) - (c.y - py) * (a.x - px)) * invArea;
                    float w2 = 1.f - w0 - w1;
                    const float slack = -0.6f / std::max(1.f, std::fabs(area));
                    if (w0 < slack || w1 < slack || w2 < slack) continue;
                    w0 = clampf(w0, 0.f, 1.f);
                    w1 = clampf(w1, 0.f, 1.f);
                    w2 = clampf(w2, 0.f, 1.f);
                    const float sum = w0 + w1 + w2;
                    if (sum < 1e-6f) continue;
                    w0 /= sum; w1 /= sum; w2 /= sum;

                    const size_t idx = static_cast<size_t>(y) * resolution + x;
                    g.position[idx] = mesh.positions[ia] * w0 + mesh.positions[ib] * w1 +
                                      mesh.positions[ic] * w2;
                    g.normal[idx] = hasNormals ? (mesh.normals[ia] * w0 + mesh.normals[ib] * w1 +
                                                  mesh.normals[ic] * w2)
                                                   .normalized()
                                               : Vec3{0.f, 1.f, 0.f};
                    g.covered[idx] = 1;
                }
            }
        }
    });
    return g;
}

// Z-buffer of the mesh as seen by one capture, used purely for visibility.
void renderDepth(const Mesh& mesh, const View& view, std::vector<float>& depth) {
    const int w = view.intrinsics.width;
    const int h = view.intrinsics.height;
    depth.assign(static_cast<size_t>(w) * h, std::numeric_limits<float>::max());

    for (size_t t = 0; t < mesh.triangleCount(); ++t) {
        Vec3 cam[3];
        Vec2 px[3];
        bool visible = true;
        for (int i = 0; i < 3; ++i) {
            cam[i] = view.worldToCamera(mesh.positions[mesh.indices[t * 3 + static_cast<size_t>(i)]]);
            if (cam[i].z <= 1e-3f) {
                visible = false;
                break;
            }
            px[i] = view.intrinsics.project(cam[i]);
        }
        if (!visible) continue;

        const float area = (px[1].x - px[0].x) * (px[2].y - px[0].y) -
                           (px[1].y - px[0].y) * (px[2].x - px[0].x);
        if (std::fabs(area) < 1e-9f) continue;
        const float invArea = 1.f / area;

        const int minX = std::max(0, static_cast<int>(std::floor(std::min(px[0].x, std::min(px[1].x, px[2].x)))));
        const int maxX = std::min(w - 1, static_cast<int>(std::ceil(std::max(px[0].x, std::max(px[1].x, px[2].x)))));
        const int minY = std::max(0, static_cast<int>(std::floor(std::min(px[0].y, std::min(px[1].y, px[2].y)))));
        const int maxY = std::min(h - 1, static_cast<int>(std::ceil(std::max(px[0].y, std::max(px[1].y, px[2].y)))));
        if (minX > maxX || minY > maxY) continue;

        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const float qx = static_cast<float>(x) + 0.5f;
                const float qy = static_cast<float>(y) + 0.5f;
                const float w0 = ((px[1].x - qx) * (px[2].y - qy) - (px[1].y - qy) * (px[2].x - qx)) * invArea;
                const float w1 = ((px[2].x - qx) * (px[0].y - qy) - (px[2].y - qy) * (px[0].x - qx)) * invArea;
                const float w2 = 1.f - w0 - w1;
                if (w0 < 0.f || w1 < 0.f || w2 < 0.f) continue;
                const float z = w0 * cam[0].z + w1 * cam[1].z + w2 * cam[2].z;
                float& slot = depth[static_cast<size_t>(y) * w + x];
                if (z < slot) slot = z;
            }
        }
    }
}

// Fills uncovered texels from their covered neighbours so that bilinear
// filtering at chart borders never samples background.
void dilate(Texture& texture, std::vector<uint8_t>& covered, int passes) {
    const int res = texture.width;
    std::vector<uint8_t> next = covered;
    for (int pass = 0; pass < passes; ++pass) {
        bool changed = false;
        for (int y = 0; y < res; ++y) {
            for (int x = 0; x < res; ++x) {
                const size_t idx = static_cast<size_t>(y) * res + x;
                if (covered[idx]) continue;
                int r = 0, g = 0, b = 0, n = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= res || ny >= res) continue;
                        const size_t nIdx = static_cast<size_t>(ny) * res + nx;
                        if (!covered[nIdx]) continue;
                        r += texture.rgb[nIdx * 3 + 0];
                        g += texture.rgb[nIdx * 3 + 1];
                        b += texture.rgb[nIdx * 3 + 2];
                        ++n;
                    }
                }
                if (n == 0) continue;
                texture.rgb[idx * 3 + 0] = static_cast<uint8_t>(r / n);
                texture.rgb[idx * 3 + 1] = static_cast<uint8_t>(g / n);
                texture.rgb[idx * 3 + 2] = static_cast<uint8_t>(b / n);
                next[idx] = 1;
                changed = true;
            }
        }
        covered = next;
        if (!changed) break;
    }
}

}  // namespace

Texture bakeTexture(const FrameStore& store, const Mesh& mesh,
                    const ReconstructionOptions& options, Progress& progress) {
    Texture texture;
    if (mesh.empty() || mesh.uvs.size() != mesh.positions.size()) return texture;

    const int res = std::max(256, options.atlasResolution);
    const size_t texels = static_cast<size_t>(res) * res;
    Gbuffer gbuffer = rasteriseAtlas(mesh, res);
    progress.report(0.15f);

    std::vector<Vec3> accum(texels, Vec3{0.f, 0.f, 0.f});
    std::vector<float> weight(texels, 0.f);

    const auto& frames = store.frames();
    const int viewCount = static_cast<int>(frames.size());
    std::vector<float> depth;
    ImageU8 color;

    for (int v = 0; v < viewCount; ++v) {
        if (progress.cancelled()) return texture;
        const Frame& frame = frames[static_cast<size_t>(v)];
        if (!store.loadColor(v, color)) continue;
        const View& view = frame.fullView;
        renderDepth(mesh, view, depth);

        const int w = view.intrinsics.width;
        const int h = view.intrinsics.height;

        parallelBlocks(res, [&](int y0, int y1) {
            for (int y = y0; y < y1; ++y) {
                for (int x = 0; x < res; ++x) {
                    const size_t idx = static_cast<size_t>(y) * res + x;
                    if (!gbuffer.covered[idx]) continue;
                    const Vec3& world = gbuffer.position[idx];
                    const Vec3 cam = view.worldToCamera(world);
                    if (cam.z <= 1e-3f) continue;
                    const Vec2 p = view.intrinsics.project(cam);
                    if (p.x < 0.5f || p.y < 0.5f || p.x > static_cast<float>(w) - 1.5f ||
                        p.y > static_cast<float>(h) - 1.5f)
                        continue;

                    const Vec3 toCamera = (view.center - world).normalized();
                    const float cosine = gbuffer.normal[idx].dot(toCamera);
                    if (cosine < options.minViewCosine) continue;

                    const float zBuffer =
                        depth[static_cast<size_t>(static_cast<int>(p.y)) * w + static_cast<int>(p.x)];
                    // Tolerance scales with depth: far surfaces have coarser z.
                    if (cam.z > zBuffer * (1.f + 0.004f) + 0.002f) continue;

                    const float wgt = std::pow(cosine, options.viewWeightExponent);
                    accum[idx] += Vec3{color.sampleBilinear(p.x, p.y, 0),
                                       color.sampleBilinear(p.x, p.y, 1),
                                       color.sampleBilinear(p.x, p.y, 2)} *
                                  wgt;
                    weight[idx] += wgt;
                }
            }
        });
        progress.report(0.15f + 0.75f * static_cast<float>(v + 1) / static_cast<float>(viewCount));
    }

    texture.width = res;
    texture.height = res;
    texture.rgb.assign(texels * 3, 0);
    std::vector<uint8_t> covered(texels, 0);
    size_t textured = 0;
    for (size_t i = 0; i < texels; ++i) {
        if (weight[i] <= 1e-6f) continue;
        const Vec3 c = accum[i] / weight[i];
        texture.rgb[i * 3 + 0] = static_cast<uint8_t>(clampf(c.x, 0.f, 255.f));
        texture.rgb[i * 3 + 1] = static_cast<uint8_t>(clampf(c.y, 0.f, 255.f));
        texture.rgb[i * 3 + 2] = static_cast<uint8_t>(clampf(c.z, 0.f, 255.f));
        covered[i] = 1;
        ++textured;
    }

    dilate(texture, covered, std::max(1, options.texturePadding));
    BC_LOGI("Texture baked: %zu / %zu texels covered", textured, texels);
    progress.report(1.f);
    return texture;
}

}  // namespace bc
