#pragma once
#include <string>

#include "surface/mesh.h"
#include "texture/texturer.h"

namespace bc {

// Writes model.obj + model.mtl + model.png next to each other.
bool writeObj(const std::string& objPath, const Mesh& mesh, const Texture& texture);

// Writes a single self-contained binary glTF with the texture embedded.
bool writeGlb(const std::string& glbPath, const Mesh& mesh, const Texture& texture);

}  // namespace bc
