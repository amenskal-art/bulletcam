#pragma once
#include <cstdint>
#include <vector>

#include "core/frame_store.h"
#include "core/progress.h"
#include "surface/mesh.h"

namespace bc {

struct Texture {
    int width = 0;
    int height = 0;
    std::vector<uint8_t> rgb;
    bool empty() const { return rgb.empty(); }
};

// Projective texture baking.
//
// Every atlas texel is resolved to a surface point, then each captured photo
// votes for its colour. Votes are weighted by how square-on the camera saw the
// surface and rejected when a per-view depth buffer says the point was
// occluded, so the far side of the object never bleeds through.
Texture bakeTexture(const FrameStore& store, const Mesh& mesh,
                    const ReconstructionOptions& options, Progress& progress);

}  // namespace bc
