#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "texture/texturer.h"

namespace bc {

// Minimal PNG encoder (8-bit RGB, Paeth-filtered, deflated with the NDK's zlib).
// Writing PNG here rather than crossing back into the JVM for Bitmap.compress
// keeps the whole export path inside the native pipeline.
bool encodePng(const Texture& texture, std::vector<uint8_t>& out);
bool writePng(const std::string& path, const Texture& texture);

}  // namespace bc
