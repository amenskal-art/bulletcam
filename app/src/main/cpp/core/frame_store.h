#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "core/camera.h"
#include "core/image.h"

namespace bc {

// Region of interest around the subject the operator framed.
//
// The capture UI already knows where the object is - the operator taps it and
// ARCore anchors it - so the reconstruction has no reason to spend time on the
// table, the wall or the room behind. Everything outside this ball is masked
// out before dense stereo runs, which both removes background geometry from the
// result and narrows the depth search to a thin shell per pixel.
struct SubjectVolume {
    Vec3 center{0.f, 0.f, 0.f};
    float radius = 0.f;  // metres; <= 0 reconstructs the whole scene

    bool enabled() const { return radius > 1e-4f; }

    bool contains(const Vec3& worldPoint) const {
        return !enabled() || (worldPoint - center).squaredNorm() <= radius * radius;
    }

    // Depth interval where the ray `origin + direction * t` crosses the ball.
    // `direction` need not be unit length, so passing a Pinhole ray (z == 1)
    // makes `t` come out directly as camera z-depth.
    bool intersect(const Vec3& origin, const Vec3& direction, float& tNear, float& tFar) const {
        const Vec3 oc = origin - center;
        const float a = direction.squaredNorm();
        if (a < 1e-12f) return false;
        const float b = oc.dot(direction);
        const float c = oc.squaredNorm() - radius * radius;
        const float discriminant = b * b - a * c;
        if (discriminant <= 0.f) return false;
        const float root = std::sqrt(discriminant);
        tNear = (-b - root) / a;
        tFar = (-b + root) / a;
        if (tFar <= 1e-4f) return false;
        if (tNear < 1e-4f) tNear = 1e-4f;
        return true;
    }
};

struct ReconstructionOptions {
    // Subject mask. Set from the anchor the operator tapped in the capture UI.
    SubjectVolume subject;
    // Longest edge of the images kept on disk for texturing.
    int textureImageMaxEdge = 1600;
    // Longest edge of the images used for dense stereo. This dominates runtime.
    int stereoImageMaxEdge = 512;
    // PatchMatch.
    int patchRadius = 4;
    int patchStep = 2;
    int patchMatchIterations = 5;
    int sourceViewsPerRef = 6;
    float minTriangulationAngleDeg = 3.f;
    float maxTriangulationAngleDeg = 55.f;
    // Depth-map filtering.
    int minConsistentViews = 2;
    float maxReprojectionError = 1.75f;
    float maxRelativeDepthError = 0.012f;
    float minNccScore = 0.35f;
    // Fusion / cleanup.
    float voxelSizeRelative = 0.0025f;  // fraction of the scene diagonal
    int outlierNeighbours = 12;
    float outlierStdDev = 2.0f;
    int normalNeighbours = 16;
    // Surface reconstruction.
    int poissonResolution = 160;
    int poissonSmoothIterations = 3;
    int poissonVCycles = 12;
    int minComponentTriangles = 400;
    int taubinIterations = 8;
    int targetTriangles = 0;  // 0 disables decimation; use poissonResolution instead
    // Texturing.
    int atlasResolution = 2048;
    int texturePadding = 6;
    float minViewCosine = 0.15f;
    float viewWeightExponent = 4.f;
};

// One captured, pose-tagged photo.
struct Frame {
    int id = 0;
    View fullView;    // intrinsics at texture resolution
    View stereoView;  // intrinsics at stereo resolution
    ImageU8 gray;     // stereo-resolution luma, kept resident (small)
    std::string rgbPath;
    int rgbWidth = 0;
    int rgbHeight = 0;
    float sharpness = 0.f;
    float depthMin = 0.f;
    float depthMax = 0.f;
};

// Owns the capture session: converts incoming YUV buffers, spills the colour
// images to the app cache directory and hands out views to the pipeline.
class FrameStore {
public:
    FrameStore(std::string workDir, const ReconstructionOptions& options);
    ~FrameStore();

    // Returns the sharpness score of the frame, or -1 if it was rejected.
    float addFrame(const uint8_t* y, const uint8_t* u, const uint8_t* v, int width, int height,
                   int yRowStride, int uvRowStride, int uvPixelStride, const float cameraToWorld[16],
                   const float intrinsics[4], float minSharpness);

    int frameCount() const;
    const std::vector<Frame>& frames() const { return frames_; }
    std::vector<Frame>& frames() { return frames_; }
    const ReconstructionOptions& options() const { return options_; }
    // Only the subject mask is meant to change after construction; the image
    // sizes are baked into the frames already on disk.
    ReconstructionOptions& mutableOptions() { return options_; }
    const std::string& workDir() const { return workDir_; }

    // Loads the texture-resolution colour image for a frame from disk.
    bool loadColor(int frameIndex, ImageU8& out) const;
    void clear();

private:
    std::string workDir_;
    ReconstructionOptions options_;
    std::vector<Frame> frames_;
    mutable std::mutex mutex_;
};

// Splits a 4x4 row-major camera-to-world matrix into rotation and centre.
void decomposeCameraToWorld(const float m[16], Mat3& Rcw, Vec3& center);

bool ensureDirectory(const std::string& path);
void removeDirectoryContents(const std::string& path);

}  // namespace bc
