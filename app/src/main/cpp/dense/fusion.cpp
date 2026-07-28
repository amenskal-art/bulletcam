#include "dense/fusion.h"

#include <mutex>

#include "core/log.h"
#include "core/parallel.h"
#include "dense/patchmatch.h"

namespace bc {

PointCloud fuseDepthMaps(const FrameStore& store,
                         const std::vector<std::vector<int>>& sourceViews,
                         std::vector<std::vector<float>>& depthMaps, Progress& progress) {
    PointCloud cloud;
    const auto& frames = store.frames();
    const int n = static_cast<int>(frames.size());
    if (n == 0) return cloud;

    const ReconstructionOptions& opt = store.options();
    const float maxReproj = opt.maxReprojectionError;
    const float maxRelDepth = opt.maxRelativeDepthError;

    // --- Geometric consistency ---------------------------------------------
    std::vector<std::vector<uint8_t>> keep(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const Frame& ref = frames[static_cast<size_t>(i)];
        keep[static_cast<size_t>(i)].assign(ref.gray.pixelCount(), 0);
    }

    for (int i = 0; i < n; ++i) {
        if (progress.cancelled()) return cloud;
        const Frame& ref = frames[static_cast<size_t>(i)];
        const View& refView = ref.stereoView;
        const std::vector<float>& refDepth = depthMaps[static_cast<size_t>(i)];
        const int w = ref.gray.width;
        const int h = ref.gray.height;
        std::vector<uint8_t>& mask = keep[static_cast<size_t>(i)];
        const std::vector<int>& sources = sourceViews[static_cast<size_t>(i)];

        parallelBlocks(h, [&](int y0, int y1) {
            for (int y = y0; y < y1; ++y) {
                for (int x = 0; x < w; ++x) {
                    const size_t idx = static_cast<size_t>(y) * w + x;
                    const float d = refDepth[idx];
                    if (d <= 1e-4f) continue;
                    const Vec3 world = refView.cameraToWorld(refView.intrinsics.unproject(
                        static_cast<float>(x), static_cast<float>(y), d));

                    int consistent = 0;
                    for (const int s : sources) {
                        const Frame& src = frames[static_cast<size_t>(s)];
                        const View& srcView = src.stereoView;
                        const Vec3 camSrc = srcView.worldToCamera(world);
                        if (camSrc.z <= 1e-4f) continue;
                        const Vec2 q = srcView.intrinsics.project(camSrc);
                        const int qx = static_cast<int>(std::lround(q.x));
                        const int qy = static_cast<int>(std::lround(q.y));
                        if (qx < 0 || qy < 0 || qx >= src.gray.width || qy >= src.gray.height)
                            continue;
                        const float ds = depthMaps[static_cast<size_t>(s)]
                                                  [static_cast<size_t>(qy) * src.gray.width + qx];
                        if (ds <= 1e-4f) continue;

                        const Vec3 backWorld = srcView.cameraToWorld(srcView.intrinsics.unproject(
                            static_cast<float>(qx), static_cast<float>(qy), ds));
                        const Vec3 backRef = refView.worldToCamera(backWorld);
                        if (backRef.z <= 1e-4f) continue;
                        const Vec2 p2 = refView.intrinsics.project(backRef);
                        const float reproj =
                            (p2 - Vec2{static_cast<float>(x), static_cast<float>(y)}).norm();
                        const float relDepth = std::fabs(backRef.z - d) / d;
                        if (reproj <= maxReproj && relDepth <= maxRelDepth) ++consistent;
                    }
                    if (consistent >= opt.minConsistentViews) mask[idx] = 1;
                }
            }
        });
        progress.report(0.5f * static_cast<float>(i + 1) / static_cast<float>(n));
    }

    // --- Unprojection ------------------------------------------------------
    std::mutex mutex;
    ImageU8 color;
    std::vector<float> normals;
    for (int i = 0; i < n; ++i) {
        if (progress.cancelled()) return cloud;
        const Frame& ref = frames[static_cast<size_t>(i)];
        const View& refView = ref.stereoView;
        const int w = ref.gray.width;
        const int h = ref.gray.height;

        char path[256];
        std::snprintf(path, sizeof(path), "%s/normal_%04d.raw", store.workDir().c_str(), ref.id);
        const bool hasNormals = loadNormalMap(path, w, h, normals);
        const bool hasColor = store.loadColor(i, color);
        const float colorScaleX =
            hasColor ? static_cast<float>(color.width) / static_cast<float>(w) : 1.f;
        const float colorScaleY =
            hasColor ? static_cast<float>(color.height) / static_cast<float>(h) : 1.f;

        PointCloud local;
        local.reserve(static_cast<size_t>(w) * h / 4);
        const std::vector<uint8_t>& mask = keep[static_cast<size_t>(i)];
        const std::vector<float>& refDepth = depthMaps[static_cast<size_t>(i)];

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                const size_t idx = static_cast<size_t>(y) * w + x;
                if (!mask[idx]) continue;
                const float d = refDepth[idx];
                const Vec3 cam = refView.intrinsics.unproject(static_cast<float>(x),
                                                              static_cast<float>(y), d);
                const Vec3 world = refView.cameraToWorld(cam);
                // Safety net for the unmasked path and for depths that drifted
                // to the very edge of the shell.
                if (!opt.subject.contains(world)) continue;

                Vec3 normalCam{0.f, 0.f, -1.f};
                if (hasNormals) {
                    normalCam = {normals[idx * 3 + 0], normals[idx * 3 + 1], normals[idx * 3 + 2]};
                    if (normalCam.squaredNorm() < 1e-6f) normalCam = -cam.normalized();
                }
                // Point the normal back along the viewing ray, i.e. outward.
                if (normalCam.dot(cam) > 0.f) normalCam = -normalCam;
                const Vec3 normalWorld = (refView.R.transpose() * normalCam).normalized();

                Vec3 rgb{0.5f, 0.5f, 0.5f};
                if (hasColor) {
                    const float cx = static_cast<float>(x) * colorScaleX;
                    const float cy = static_cast<float>(y) * colorScaleY;
                    rgb = {color.sampleBilinear(cx, cy, 0) / 255.f,
                           color.sampleBilinear(cx, cy, 1) / 255.f,
                           color.sampleBilinear(cx, cy, 2) / 255.f};
                }
                local.push(world, normalWorld, rgb);
            }
        }

        std::lock_guard<std::mutex> lock(mutex);
        cloud.positions.insert(cloud.positions.end(), local.positions.begin(),
                               local.positions.end());
        cloud.normals.insert(cloud.normals.end(), local.normals.begin(), local.normals.end());
        cloud.colors.insert(cloud.colors.end(), local.colors.begin(), local.colors.end());
        progress.report(0.5f + 0.5f * static_cast<float>(i + 1) / static_cast<float>(n));
    }

    BC_LOGI("Fusion produced %zu raw points", cloud.size());
    return cloud;
}

}  // namespace bc
