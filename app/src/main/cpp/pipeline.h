#pragma once
#include <string>

#include "core/frame_store.h"
#include "core/progress.h"

namespace bc {

struct ReconstructionResult {
    bool success = false;
    std::string message;
    int pointCount = 0;
    int triangleCount = 0;
};

// Runs the whole offline reconstruction: sparse alignment, dense stereo,
// fusion, Poisson surfacing, UV unwrap, texture bake and export.
ReconstructionResult runReconstruction(FrameStore& store, const std::string& objPath,
                                       const std::string& glbPath, Progress& progress);

}  // namespace bc
