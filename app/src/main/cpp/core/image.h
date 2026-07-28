#pragma once
#include <cstring>
#include <string>
#include <vector>

#include "core/types.h"

namespace bc {

template <typename T>
struct Image {
    int width = 0;
    int height = 0;
    int channels = 0;
    std::vector<T> data;

    Image() = default;
    Image(int w, int h, int c) { resize(w, h, c); }

    void resize(int w, int h, int c) {
        width = w;
        height = h;
        channels = c;
        data.assign(static_cast<size_t>(w) * h * c, T(0));
    }
    bool empty() const { return data.empty(); }
    size_t pixelCount() const { return static_cast<size_t>(width) * height; }

    T* ptr(int y) { return data.data() + static_cast<size_t>(y) * width * channels; }
    const T* ptr(int y) const { return data.data() + static_cast<size_t>(y) * width * channels; }
    T& at(int x, int y, int c = 0) {
        return data[(static_cast<size_t>(y) * width + x) * channels + c];
    }
    const T& at(int x, int y, int c = 0) const {
        return data[(static_cast<size_t>(y) * width + x) * channels + c];
    }
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
    bool inside(float x, float y) const {
        return x >= 0.f && y >= 0.f && x <= static_cast<float>(width - 1) &&
               y <= static_cast<float>(height - 1);
    }

    float sampleBilinear(float x, float y, int c = 0) const {
        if (x < 0.f) x = 0.f;
        if (y < 0.f) y = 0.f;
        if (x > static_cast<float>(width - 1)) x = static_cast<float>(width - 1);
        if (y > static_cast<float>(height - 1)) y = static_cast<float>(height - 1);
        const int x0 = static_cast<int>(x);
        const int y0 = static_cast<int>(y);
        const int x1 = std::min(x0 + 1, width - 1);
        const int y1 = std::min(y0 + 1, height - 1);
        const float fx = x - static_cast<float>(x0);
        const float fy = y - static_cast<float>(y0);
        const float v00 = static_cast<float>(at(x0, y0, c));
        const float v10 = static_cast<float>(at(x1, y0, c));
        const float v01 = static_cast<float>(at(x0, y1, c));
        const float v11 = static_cast<float>(at(x1, y1, c));
        return (v00 * (1.f - fx) + v10 * fx) * (1.f - fy) +
               (v01 * (1.f - fx) + v11 * fx) * fy;
    }
};

using ImageU8 = Image<uint8_t>;
using ImageF = Image<float>;

// Converts an Android YUV_420_888 triple-plane image into interleaved RGB and
// a single-channel luma image, reading straight from the direct ByteBuffer
// addresses handed over by JNI (no intermediate copies).
void yuv420ToRgbAndGray(const uint8_t* yPlane, const uint8_t* uPlane, const uint8_t* vPlane,
                        int width, int height, int yRowStride, int uvRowStride, int uvPixelStride,
                        ImageU8& rgbOut, ImageU8& grayOut);

// Area-averaged downscale. Works for any channel count.
ImageU8 downscale(const ImageU8& src, int dstWidth, int dstHeight);

// Variance of the Laplacian: a cheap and effective blur score.
float sharpnessScore(const ImageU8& gray);

bool writeRawImage(const std::string& path, const ImageU8& img);
bool readRawImage(const std::string& path, int width, int height, int channels, ImageU8& out);

}  // namespace bc
