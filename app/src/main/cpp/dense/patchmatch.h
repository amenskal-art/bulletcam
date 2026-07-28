#pragma once
#include <string>
#include <vector>

#include "core/frame_store.h"
#include "sparse/sparse_map.h"

namespace bc {

// Per-pixel depth, surface normal (reference camera frame) and matching cost.
struct DepthMap {
    int width = 0;
    int height = 0;
    std::vector<float> depth;   // 0 == no hypothesis
    std::vector<float> normal;  // 3 floats per pixel
    std::vector<float> cost;    // 1 - NCC, aggregated over source views

    void resize(int w, int h) {
        width = w;
        height = h;
        depth.assign(static_cast<size_t>(w) * h, 0.f);
        normal.assign(static_cast<size_t>(w) * h * 3, 0.f);
        cost.assign(static_cast<size_t>(w) * h, 3.f);
    }
};

// PatchMatch multi-view stereo. ARCore already supplies the extrinsics, so this
// skips structure-from-motion entirely and sweeps plane hypotheses directly,
// using the plane-induced homography between reference and source views.
void computeDepthMap(const FrameStore& store, const SparseMap& sparse, int refIndex,
                     const std::vector<int>& sourceViews, DepthMap& out);

bool saveNormalMap(const std::string& path, const DepthMap& map);
bool loadNormalMap(const std::string& path, int width, int height, std::vector<float>& normals);

}  // namespace bc
