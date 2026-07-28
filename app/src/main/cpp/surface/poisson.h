#pragma once
#include "core/frame_store.h"
#include "core/progress.h"
#include "surface/grid.h"
#include "surface/mesh.h"
#include "surface/point_cloud.h"

namespace bc {

// Poisson surface reconstruction on a uniform grid.
//
// Follows Kazhdan's formulation: splat the oriented samples into a smoothed
// vector field V, then solve the Poisson equation  laplacian(chi) = div(V)
// for the indicator function chi and extract the level set at the average value
// chi takes on the input samples. The octree of the original paper is replaced
// by a regular grid solved with a geometric multigrid V-cycle, which is a much
// better fit for a phone: fixed memory, no pointer chasing, trivially threaded.
Mesh poissonReconstruct(const PointCloud& cloud, const ReconstructionOptions& options,
                        Progress& progress);

}  // namespace bc
