#pragma once
#include "surface/mesh.h"

namespace bc {

// Assigns a UV parameterisation to the mesh, splitting vertices where charts
// require it. Returns false only if the mesh has no triangles.
//
// With BULLETCAM_HAVE_XATLAS the work is handed to xatlas. The built-in
// fallback grows near-planar charts across the mesh, projects each one onto its
// own average plane and shelf packs them - more seams and more distortion than
// xatlas, but no external dependency and a completely offline build.
bool buildAtlas(Mesh& mesh, int atlasResolution);

}  // namespace bc
