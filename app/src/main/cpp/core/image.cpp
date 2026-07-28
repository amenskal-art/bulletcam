#include "core/image.h"

#include <cstdio>

#include "core/parallel.h"

namespace bc {

void yuv420ToRgbAndGray(const uint8_t* yPlane, const uint8_t* uPlane, const uint8_t* vPlane,
                        int width, int height, int yRowStride, int uvRowStride, int uvPixelStride,
                        ImageU8& rgbOut, ImageU8& grayOut) {
    rgbOut.resize(width, height, 3);
    grayOut.resize(width, height, 1);

    parallelBlocks(height, [&](int y0, int y1) {
        for (int y = y0; y < y1; ++y) {
            const uint8_t* yRow = yPlane + static_cast<size_t>(y) * yRowStride;
            const uint8_t* uRow = uPlane + static_cast<size_t>(y >> 1) * uvRowStride;
            const uint8_t* vRow = vPlane + static_cast<size_t>(y >> 1) * uvRowStride;
            uint8_t* rgb = rgbOut.ptr(y);
            uint8_t* gray = grayOut.ptr(y);
            for (int x = 0; x < width; ++x) {
                const int luma = yRow[x];
                gray[x] = static_cast<uint8_t>(luma);
                const int uvIndex = (x >> 1) * uvPixelStride;
                const int cb = static_cast<int>(uRow[uvIndex]) - 128;
                const int cr = static_cast<int>(vRow[uvIndex]) - 128;
                // ITU-R BT.601 video range, fixed point (16.16).
                const int yy = (luma - 16) * 76309;
                int r = (yy + 104597 * cr) >> 16;
                int g = (yy - 25675 * cb - 53279 * cr) >> 16;
                int b = (yy + 132201 * cb) >> 16;
                rgb[x * 3 + 0] = static_cast<uint8_t>(r < 0 ? 0 : (r > 255 ? 255 : r));
                rgb[x * 3 + 1] = static_cast<uint8_t>(g < 0 ? 0 : (g > 255 ? 255 : g));
                rgb[x * 3 + 2] = static_cast<uint8_t>(b < 0 ? 0 : (b > 255 ? 255 : b));
            }
        }
    });
}

ImageU8 downscale(const ImageU8& src, int dstWidth, int dstHeight) {
    ImageU8 dst(dstWidth, dstHeight, src.channels);
    if (src.empty() || dstWidth <= 0 || dstHeight <= 0) return dst;
    const float sx = static_cast<float>(src.width) / static_cast<float>(dstWidth);
    const float sy = static_cast<float>(src.height) / static_cast<float>(dstHeight);
    const int c = src.channels;

    parallelBlocks(dstHeight, [&](int y0, int y1) {
        std::vector<int> acc(static_cast<size_t>(c));
        for (int y = y0; y < y1; ++y) {
            const int srcY0 = static_cast<int>(y * sy);
            const int srcY1 = std::max(srcY0 + 1, std::min(src.height, static_cast<int>((y + 1) * sy)));
            for (int x = 0; x < dstWidth; ++x) {
                const int srcX0 = static_cast<int>(x * sx);
                const int srcX1 = std::max(srcX0 + 1, std::min(src.width, static_cast<int>((x + 1) * sx)));
                std::fill(acc.begin(), acc.end(), 0);
                int n = 0;
                for (int yy = srcY0; yy < srcY1; ++yy) {
                    const uint8_t* row = src.ptr(yy);
                    for (int xx = srcX0; xx < srcX1; ++xx) {
                        for (int ch = 0; ch < c; ++ch) acc[ch] += row[xx * c + ch];
                        ++n;
                    }
                }
                uint8_t* out = dst.ptr(y) + x * c;
                for (int ch = 0; ch < c; ++ch) out[ch] = static_cast<uint8_t>(acc[ch] / std::max(1, n));
            }
        }
    });
    return dst;
}

float sharpnessScore(const ImageU8& gray) {
    if (gray.width < 8 || gray.height < 8) return 0.f;
    double sum = 0.0, sumSq = 0.0;
    long count = 0;
    // Sub-sample: a stride of 2 keeps the estimate stable and costs a quarter of the work.
    for (int y = 1; y < gray.height - 1; y += 2) {
        for (int x = 1; x < gray.width - 1; x += 2) {
            const float lap = -4.f * gray.at(x, y) + gray.at(x - 1, y) + gray.at(x + 1, y) +
                              gray.at(x, y - 1) + gray.at(x, y + 1);
            sum += lap;
            sumSq += static_cast<double>(lap) * lap;
            ++count;
        }
    }
    if (count < 2) return 0.f;
    const double mean = sum / static_cast<double>(count);
    return static_cast<float>(sumSq / static_cast<double>(count) - mean * mean);
}

bool writeRawImage(const std::string& path, const ImageU8& img) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const size_t n = img.data.size();
    const bool ok = n == 0 || std::fwrite(img.data.data(), 1, n, f) == n;
    std::fclose(f);
    return ok;
}

bool readRawImage(const std::string& path, int width, int height, int channels, ImageU8& out) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    out.resize(width, height, channels);
    const size_t n = out.data.size();
    const bool ok = std::fread(out.data.data(), 1, n, f) == n;
    std::fclose(f);
    return ok;
}

}  // namespace bc
