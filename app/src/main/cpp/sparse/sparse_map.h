#pragma once
#include <vector>

#include "core/frame_store.h"
#include "core/progress.h"
#include "sparse/orb.h"

namespace bc {

struct SparsePoint {
    Vec3 position;
    int viewA = -1;
    int viewB = -1;
    float error = 0.f;  // mean reprojection error in pixels
};

// The result of the sparse stage: a lightweight point cloud used to seed the
// dense stereo (per-view depth priors) and to score the view graph.
struct SparseMap {
    std::vector<SparsePoint> points;
    // sharedPoints[i * n + j] = number of triangulated points shared by views i and j.
    std::vector<int> sharedPoints;
    int viewCount = 0;
    BBox bounds;
    Vec3 centroid;

    int shared(int i, int j) const {
        return sharedPoints[static_cast<size_t>(i) * viewCount + j];
    }
};

// Triangulates a pair of pixel observations by the midpoint of the common
// perpendicular of the two viewing rays. Returns false for degenerate baselines,
// points behind either camera, or angles below `minAngleDeg`.
bool triangulateRays(const View& a, const View& b, const Vec2& pa, const Vec2& pb,
                     float minAngleDeg, float maxAngleDeg, Vec3& outPoint, float& outAngleDeg);

// Runs feature extraction, epipolar-guided matching against pose-nearest
// neighbours and ray triangulation. Also fills each frame's depth range.
SparseMap buildSparseMap(FrameStore& store, Progress& progress);

}  // namespace bc
