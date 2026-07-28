#include "pipeline.h"

#include <chrono>

#include "core/log.h"
#include "dense/fusion.h"
#include "dense/patchmatch.h"
#include "dense/view_selection.h"
#include "io/model_writer.h"
#include "sparse/sparse_map.h"
#include "surface/atlas.h"
#include "surface/mesh.h"
#include "surface/poisson.h"
#include "texture/texturer.h"

namespace bc {
namespace {

double elapsedSeconds(const std::chrono::steady_clock::time_point& start) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

ReconstructionResult failure(const std::string& message) {
    ReconstructionResult result;
    result.success = false;
    result.message = message;
    return result;
}

}  // namespace

ReconstructionResult runReconstruction(FrameStore& store, const std::string& objPath,
                                       const std::string& glbPath, Progress& progress) {
    const auto start = std::chrono::steady_clock::now();
    const ReconstructionOptions& options = store.options();
    const int viewCount = store.frameCount();
    if (viewCount < 8)
        return failure("Capture at least 8 photos from different angles before reconstructing.");

    if (options.subject.enabled()) {
        BC_LOGI("Subject mask: r=%.3f m at (%.3f, %.3f, %.3f)", options.subject.radius,
                options.subject.center.x, options.subject.center.y, options.subject.center.z);
    }

    // --- 1. Sparse alignment ------------------------------------------------
    progress.setStage("Aligning captures", 0.f, 0.12f);
    SparseMap sparse = buildSparseMap(store, progress);
    if (progress.cancelled()) return failure("Cancelled");
    if (sparse.points.size() < 200)
        return failure(
            options.subject.enabled()
                ? "Not enough detail inside the capture volume. Grow the volume so it covers the "
                  "whole object, or orbit more slowly."
                : "Not enough overlap between photos. Orbit more slowly and keep the object "
                  "filling the frame.");
    BC_LOGI("Sparse alignment: %zu points in %.1fs", sparse.points.size(), elapsedSeconds(start));

    // --- 2. View graph ------------------------------------------------------
    progress.setStage("Selecting views", 0.12f, 0.14f);
    const std::vector<std::vector<int>> sourceViews =
        selectSourceViews(store, sparse, options.sourceViewsPerRef);
    progress.report(1.f);

    // --- 3. Dense stereo ----------------------------------------------------
    progress.setStage("Dense stereo matching", 0.14f, 0.62f);
    std::vector<std::vector<float>> depthMaps(static_cast<size_t>(viewCount));
    for (int i = 0; i < viewCount; ++i) {
        if (progress.cancelled()) return failure("Cancelled");
        const std::vector<int>& sources = sourceViews[static_cast<size_t>(i)];
        const Frame& frame = store.frames()[static_cast<size_t>(i)];
        if (sources.empty()) {
            depthMaps[static_cast<size_t>(i)].assign(frame.gray.pixelCount(), 0.f);
            continue;
        }
        DepthMap map;
        computeDepthMap(store, sparse, i, sources, map);

        char path[256];
        std::snprintf(path, sizeof(path), "%s/normal_%04d.raw", store.workDir().c_str(), frame.id);
        saveNormalMap(path, map);
        depthMaps[static_cast<size_t>(i)] = std::move(map.depth);
        progress.report(static_cast<float>(i + 1) / static_cast<float>(viewCount));
    }
    BC_LOGI("Dense stereo finished at %.1fs", elapsedSeconds(start));

    // --- 4. Consistency filtering and fusion --------------------------------
    progress.setStage("Fusing depth maps", 0.62f, 0.74f);
    PointCloud cloud = fuseDepthMaps(store, sourceViews, depthMaps, progress);
    depthMaps.clear();
    depthMaps.shrink_to_fit();
    if (progress.cancelled()) return failure("Cancelled");
    if (cloud.size() < 5000)
        return failure(
            "The dense stage recovered too little surface. Try brighter, more even lighting and a "
            "textured background.");

    // --- 5. Point cloud clean-up -------------------------------------------
    progress.setStage("Cleaning point cloud", 0.74f, 0.78f);
    const BBox cloudBounds = computeBounds(cloud);
    const float voxel = std::max(1e-5f, cloudBounds.maxExtent() * options.voxelSizeRelative);
    voxelDownsample(cloud, voxel);
    progress.report(0.4f);
    removeStatisticalOutliers(cloud, options.outlierNeighbours, options.outlierStdDev);
    progress.report(0.7f);
    refineNormals(cloud, options.normalNeighbours);
    progress.report(1.f);
    BC_LOGI("Point cloud: %zu points after clean-up (voxel %.4f m)", cloud.size(), voxel);
    if (cloud.size() < 2000) return failure("Point cloud collapsed during filtering.");

    // --- 6. Surface reconstruction -----------------------------------------
    progress.setStage("Building surface", 0.78f, 0.90f);
    Mesh mesh = poissonReconstruct(cloud, options, progress);
    if (progress.cancelled()) return failure("Cancelled");
    if (mesh.empty()) return failure("Surface reconstruction produced no geometry.");

    removeSmallComponents(mesh, options.minComponentTriangles);
    taubinSmooth(mesh, options.taubinIterations, 0.5f, -0.53f);
    if (options.targetTriangles > 0) decimate(mesh, options.targetTriangles);
    computeVertexNormals(mesh);
    if (mesh.empty()) return failure("Surface reconstruction produced no geometry.");
    BC_LOGI("Mesh after clean-up: %zu triangles", mesh.triangleCount());

    // --- 7. UV unwrap -------------------------------------------------------
    progress.setStage("Unwrapping UVs", 0.90f, 0.93f);
    if (!buildAtlas(mesh, options.atlasResolution))
        BC_LOGW("UV unwrapping failed; exporting untextured geometry");
    progress.report(1.f);

    // --- 8. Texture bake ----------------------------------------------------
    progress.setStage("Baking texture", 0.93f, 0.98f);
    Texture texture = bakeTexture(store, mesh, options, progress);
    if (progress.cancelled()) return failure("Cancelled");

    // --- 9. Export ----------------------------------------------------------
    progress.setStage("Writing model", 0.98f, 1.f);
    if (!writeObj(objPath, mesh, texture)) return failure("Could not write the OBJ model.");
    if (!writeGlb(glbPath, mesh, texture)) return failure("Could not write the GLB model.");
    progress.report(1.f);

    ReconstructionResult result;
    result.success = true;
    result.pointCount = static_cast<int>(cloud.size());
    result.triangleCount = static_cast<int>(mesh.triangleCount());
    char message[192];
    std::snprintf(message, sizeof(message), "%d triangles from %d photos in %.0f s",
                  result.triangleCount, viewCount, elapsedSeconds(start));
    result.message = message;
    BC_LOGI("Reconstruction complete: %s", result.message.c_str());
    return result;
}

}  // namespace bc
