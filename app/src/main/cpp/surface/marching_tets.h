#pragma once
#include <vector>

#include "surface/grid.h"
#include "surface/mesh.h"

namespace bc {

// Marching tetrahedra over the implicit surface {chi == iso}.
// Each cube is split into six tetrahedra sharing the 0-6 diagonal, which needs
// only a sixteen-case table and cannot produce the ambiguous configurations
// that plain marching cubes has to disambiguate.
// `vertexMask` (grid-vertex sized, may be empty) restricts extraction to cells
// close to real samples so the solver's far field never becomes geometry.
Mesh extractIsosurface(const ScalarGrid& grid, float iso, const std::vector<uint8_t>& vertexMask);

}  // namespace bc
