#pragma once
#include <vector>

#include "core/frame_store.h"
#include "sparse/sparse_map.h"

namespace bc {

// For every reference view, the ordered list of source views used for
// plane-sweep matching. Scoring follows the COLMAP heuristic: co-visibility
// weighted by a preferred triangulation angle.
std::vector<std::vector<int>> selectSourceViews(const FrameStore& store, const SparseMap& sparse,
                                                int sourcesPerView);

}  // namespace bc
