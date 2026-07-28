#include "io/png.h"

#include <zlib.h>

#include <cstdio>
#include <cstring>

namespace bc {
namespace {

void appendBigEndian32(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>(value & 0xFF));
}

void appendChunk(std::vector<uint8_t>& out, const char type[4], const uint8_t* data, size_t length) {
    appendBigEndian32(out, static_cast<uint32_t>(length));
    const size_t crcStart = out.size();
    out.insert(out.end(), type, type + 4);
    if (length > 0) out.insert(out.end(), data, data + length);
    const uLong crc = crc32(crc32(0L, Z_NULL, 0), out.data() + crcStart,
                            static_cast<uInt>(out.size() - crcStart));
    appendBigEndian32(out, static_cast<uint32_t>(crc));
}

uint8_t paethPredictor(int a, int b, int c) {
    const int p = a + b - c;
    const int pa = std::abs(p - a);
    const int pb = std::abs(p - b);
    const int pc = std::abs(p - c);
    if (pa <= pb && pa <= pc) return static_cast<uint8_t>(a);
    if (pb <= pc) return static_cast<uint8_t>(b);
    return static_cast<uint8_t>(c);
}

}  // namespace

bool encodePng(const Texture& texture, std::vector<uint8_t>& out) {
    if (texture.empty() || texture.width <= 0 || texture.height <= 0) return false;
    constexpr int kBytesPerPixel = 3;
    const size_t stride = static_cast<size_t>(texture.width) * kBytesPerPixel;

    // Filtered scanlines: one filter byte per row, Paeth for all of them.
    std::vector<uint8_t> raw((stride + 1) * static_cast<size_t>(texture.height));
    for (int y = 0; y < texture.height; ++y) {
        uint8_t* dst = raw.data() + static_cast<size_t>(y) * (stride + 1);
        dst[0] = 4;  // Paeth
        const uint8_t* row = texture.rgb.data() + static_cast<size_t>(y) * stride;
        const uint8_t* prev = y > 0 ? texture.rgb.data() + static_cast<size_t>(y - 1) * stride : nullptr;
        for (size_t i = 0; i < stride; ++i) {
            const int a = i >= kBytesPerPixel ? row[i - kBytesPerPixel] : 0;
            const int b = prev ? prev[i] : 0;
            const int c = (prev && i >= kBytesPerPixel) ? prev[i - kBytesPerPixel] : 0;
            dst[1 + i] = static_cast<uint8_t>(row[i] - paethPredictor(a, b, c));
        }
    }

    uLongf compressedSize = compressBound(static_cast<uLong>(raw.size()));
    std::vector<uint8_t> compressed(compressedSize);
    if (compress2(compressed.data(), &compressedSize, raw.data(), static_cast<uLong>(raw.size()), 6) !=
        Z_OK)
        return false;
    compressed.resize(compressedSize);

    out.clear();
    out.reserve(compressed.size() + 128);
    const uint8_t signature[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    out.insert(out.end(), signature, signature + 8);

    uint8_t ihdr[13];
    ihdr[0] = static_cast<uint8_t>((texture.width >> 24) & 0xFF);
    ihdr[1] = static_cast<uint8_t>((texture.width >> 16) & 0xFF);
    ihdr[2] = static_cast<uint8_t>((texture.width >> 8) & 0xFF);
    ihdr[3] = static_cast<uint8_t>(texture.width & 0xFF);
    ihdr[4] = static_cast<uint8_t>((texture.height >> 24) & 0xFF);
    ihdr[5] = static_cast<uint8_t>((texture.height >> 16) & 0xFF);
    ihdr[6] = static_cast<uint8_t>((texture.height >> 8) & 0xFF);
    ihdr[7] = static_cast<uint8_t>(texture.height & 0xFF);
    ihdr[8] = 8;   // bit depth
    ihdr[9] = 2;   // colour type: truecolour
    ihdr[10] = 0;  // compression
    ihdr[11] = 0;  // filter
    ihdr[12] = 0;  // interlace
    appendChunk(out, "IHDR", ihdr, sizeof(ihdr));
    appendChunk(out, "IDAT", compressed.data(), compressed.size());
    appendChunk(out, "IEND", nullptr, 0);
    return true;
}

bool writePng(const std::string& path, const Texture& texture) {
    std::vector<uint8_t> bytes;
    if (!encodePng(texture, bytes)) return false;
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(bytes.data(), 1, bytes.size(), f) == bytes.size();
    std::fclose(f);
    return ok;
}

}  // namespace bc
