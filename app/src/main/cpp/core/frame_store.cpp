#include "core/frame_store.h"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>

#include "core/log.h"

namespace bc {

void decomposeCameraToWorld(const float m[16], Mat3& Rcw, Vec3& center) {
    Rcw(0, 0) = m[0];  Rcw(0, 1) = m[1];  Rcw(0, 2) = m[2];
    Rcw(1, 0) = m[4];  Rcw(1, 1) = m[5];  Rcw(1, 2) = m[6];
    Rcw(2, 0) = m[8];  Rcw(2, 1) = m[9];  Rcw(2, 2) = m[10];
    Rcw = Rcw.orthonormalized();
    center = Vec3{m[3], m[7], m[11]};
}

bool ensureDirectory(const std::string& path) {
    struct stat st {};
    if (stat(path.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
    return mkdir(path.c_str(), 0770) == 0;
}

void removeDirectoryContents(const std::string& path) {
    DIR* dir = opendir(path.c_str());
    if (!dir) return;
    while (dirent* entry = readdir(dir)) {
        const std::string name = entry->d_name;
        if (name == "." || name == "..") continue;
        const std::string full = path + "/" + name;
        struct stat st {};
        if (stat(full.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) {
            removeDirectoryContents(full);
            rmdir(full.c_str());
        } else {
            unlink(full.c_str());
        }
    }
    closedir(dir);
}

namespace {
void fitInside(int width, int height, int maxEdge, int& outW, int& outH) {
    const int longEdge = std::max(width, height);
    if (longEdge <= maxEdge) {
        outW = width;
        outH = height;
        return;
    }
    const float scale = static_cast<float>(maxEdge) / static_cast<float>(longEdge);
    outW = std::max(1, static_cast<int>(std::lround(width * scale)));
    outH = std::max(1, static_cast<int>(std::lround(height * scale)));
}
}  // namespace

FrameStore::FrameStore(std::string workDir, const ReconstructionOptions& options)
    : workDir_(std::move(workDir)), options_(options) {
    ensureDirectory(workDir_);
    removeDirectoryContents(workDir_);
}

FrameStore::~FrameStore() = default;

int FrameStore::frameCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(frames_.size());
}

float FrameStore::addFrame(const uint8_t* y, const uint8_t* u, const uint8_t* v, int width,
                           int height, int yRowStride, int uvRowStride, int uvPixelStride,
                           const float cameraToWorld[16], const float intrinsics[4],
                           float minSharpness) {
    ImageU8 rgb, gray;
    yuv420ToRgbAndGray(y, u, v, width, height, yRowStride, uvRowStride, uvPixelStride, rgb, gray);

    int stereoW = 0, stereoH = 0;
    fitInside(width, height, options_.stereoImageMaxEdge, stereoW, stereoH);
    ImageU8 grayLow = downscale(gray, stereoW, stereoH);

    const float score = sharpnessScore(grayLow);
    if (score < minSharpness) return -1.f;

    int texW = 0, texH = 0;
    fitInside(width, height, options_.textureImageMaxEdge, texW, texH);
    ImageU8 rgbTex = (texW == width && texH == height) ? rgb : downscale(rgb, texW, texH);

    Pinhole fullK;
    fullK.fx = intrinsics[0];
    fullK.fy = intrinsics[1];
    fullK.cx = intrinsics[2];
    fullK.cy = intrinsics[3];
    fullK.width = width;
    fullK.height = height;

    Mat3 Rcw;
    Vec3 center;
    decomposeCameraToWorld(cameraToWorld, Rcw, center);

    Frame frame;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frame.id = static_cast<int>(frames_.size());
    }
    frame.fullView.intrinsics = fullK.scaled(texW, texH);
    frame.fullView.setFromCameraToWorld(Rcw, center);
    frame.stereoView.intrinsics = fullK.scaled(stereoW, stereoH);
    frame.stereoView.setFromCameraToWorld(Rcw, center);
    frame.gray = std::move(grayLow);
    frame.rgbWidth = texW;
    frame.rgbHeight = texH;
    frame.sharpness = score;

    char name[64];
    std::snprintf(name, sizeof(name), "/rgb_%04d.raw", frame.id);
    frame.rgbPath = workDir_ + name;
    if (!writeRawImage(frame.rgbPath, rgbTex)) {
        BC_LOGE("Failed to spill colour image to %s", frame.rgbPath.c_str());
        return -1.f;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.push_back(std::move(frame));
    }
    return score;
}

bool FrameStore::loadColor(int frameIndex, ImageU8& out) const {
    if (frameIndex < 0 || frameIndex >= static_cast<int>(frames_.size())) return false;
    const Frame& f = frames_[static_cast<size_t>(frameIndex)];
    return readRawImage(f.rgbPath, f.rgbWidth, f.rgbHeight, 3, out);
}

void FrameStore::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.clear();
    removeDirectoryContents(workDir_);
}

}  // namespace bc
