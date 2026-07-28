#pragma once
#include <vector>

#include "core/frame_store.h"
#include "core/progress.h"
#include "surface/point_cloud.h"

namespace bc {

// Cross-checks every depth hypothesis against its source views and keeps only
// the ones that survive forward-backward reprojection, then unprojects the
// survivors into a single oriented, coloured point cloud.
PointCloud fuseDepthMaps(const FrameStore& store,
                         const std::vector<std::vector<int>>& sourceViews,
                         std::vector<std::vector<float>>& depthMaps, Progress& progress);

}  // namespace bc
