// Host-side integration test for the reconstruction core.
//
// Renders a textured sphere from a synthetic orbit, feeds the frames through
// the same FrameStore -> pipeline path the app uses, and checks that a mesh
// comes out. Building the CV core off-device makes it possible to iterate on
// the geometry without a phone in the loop.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "core/frame_store.h"
#include "pipeline.h"

using namespace bc;

namespace {

constexpr int kImageWidth = 480;
constexpr int kImageHeight = 360;
constexpr float kSphereRadius = 0.15f;
constexpr float kOrbitRadius = 0.55f;

uint32_t hash3(int x, int y, int z) {
    uint32_t h = static_cast<uint32_t>(x) * 73856093u ^ static_cast<uint32_t>(y) * 19349663u ^
                 static_cast<uint32_t>(z) * 83492791u;
    h ^= h >> 13;
    h *= 0x5bd1e995u;
    h ^= h >> 15;
    return h;
}

// High frequency surface texture so the matcher has something to lock onto.
void surfaceColor(const Vec3& p, uint8_t rgb[3]) {
    const int cx = static_cast<int>(std::floor(p.x * 260.f));
    const int cy = static_cast<int>(std::floor(p.y * 260.f));
    const int cz = static_cast<int>(std::floor(p.z * 260.f));
    const uint32_t h = hash3(cx, cy, cz);
    const float base = 45.f + static_cast<float>(h & 0xFF) * 0.72f;
    rgb[0] = static_cast<uint8_t>(clampf(base + 22.f * std::sin(p.x * 30.f), 0.f, 255.f));
    rgb[1] = static_cast<uint8_t>(clampf(base + 18.f * std::sin(p.y * 27.f), 0.f, 255.f));
    rgb[2] = static_cast<uint8_t>(clampf(base + 25.f * std::sin(p.z * 33.f), 0.f, 255.f));
}

struct RenderedFrame {
    std::vector<uint8_t> y, u, v;
    float cameraToWorld[16];
    float intrinsics[4];
};

RenderedFrame render(float azimuthDeg, float elevationDeg) {
    const float az = azimuthDeg * 0.0174532925f;
    const float el = elevationDeg * 0.0174532925f;
    const Vec3 center{kOrbitRadius * std::cos(el) * std::cos(az), kOrbitRadius * std::sin(el),
                      kOrbitRadius * std::cos(el) * std::sin(az)};

    const Vec3 forward = (-center).normalized();
    Vec3 worldUp{0.f, 1.f, 0.f};
    if (std::fabs(forward.dot(worldUp)) > 0.98f) worldUp = Vec3{1.f, 0.f, 0.f};
    const Vec3 right = worldUp.cross(forward).normalized();
    const Vec3 down = forward.cross(right);

    RenderedFrame frame;
    const float mat[16] = {right.x, down.x, forward.x, center.x, right.y, down.y, forward.y,
                           center.y, right.z, down.z,  forward.z, center.z, 0.f,  0.f,
                           0.f,      1.f};
    std::memcpy(frame.cameraToWorld, mat, sizeof(mat));

    Pinhole k;
    k.fx = 520.f;
    k.fy = 520.f;
    k.cx = kImageWidth * 0.5f - 0.5f;
    k.cy = kImageHeight * 0.5f - 0.5f;
    k.width = kImageWidth;
    k.height = kImageHeight;
    frame.intrinsics[0] = k.fx;
    frame.intrinsics[1] = k.fy;
    frame.intrinsics[2] = k.cx;
    frame.intrinsics[3] = k.cy;

    Mat3 rotation;
    rotation(0, 0) = right.x; rotation(0, 1) = down.x; rotation(0, 2) = forward.x;
    rotation(1, 0) = right.y; rotation(1, 1) = down.y; rotation(1, 2) = forward.y;
    rotation(2, 0) = right.z; rotation(2, 1) = down.z; rotation(2, 2) = forward.z;

    frame.y.assign(static_cast<size_t>(kImageWidth) * kImageHeight, 16);
    frame.u.assign(static_cast<size_t>(kImageWidth / 2) * (kImageHeight / 2), 128);
    frame.v.assign(static_cast<size_t>(kImageWidth / 2) * (kImageHeight / 2), 128);

    for (int py = 0; py < kImageHeight; ++py) {
        for (int px = 0; px < kImageWidth; ++px) {
            const Vec3 dir = (rotation * k.ray(static_cast<float>(px), static_cast<float>(py))).normalized();
            // Ray / sphere intersection, sphere centred at the origin.
            const float b = 2.f * dir.dot(center);
            const float c = center.squaredNorm() - kSphereRadius * kSphereRadius;
            const float disc = b * b - 4.f * c;
            uint8_t rgb[3] = {8, 8, 10};
            if (disc > 0.f) {
                const float t = (-b - std::sqrt(disc)) * 0.5f;
                if (t > 0.f) surfaceColor(center + dir * t, rgb);
            }
            const int luma = (66 * rgb[0] + 129 * rgb[1] + 25 * rgb[2] + 128) / 256 + 16;
            const int cb = (-38 * rgb[0] - 74 * rgb[1] + 112 * rgb[2] + 128) / 256 + 128;
            const int cr = (112 * rgb[0] - 94 * rgb[1] - 18 * rgb[2] + 128) / 256 + 128;
            frame.y[static_cast<size_t>(py) * kImageWidth + px] =
                static_cast<uint8_t>(clampf(static_cast<float>(luma), 0.f, 255.f));
            if ((px % 2) == 0 && (py % 2) == 0) {
                const size_t idx = static_cast<size_t>(py / 2) * (kImageWidth / 2) + (px / 2);
                frame.u[idx] = static_cast<uint8_t>(clampf(static_cast<float>(cb), 0.f, 255.f));
                frame.v[idx] = static_cast<uint8_t>(clampf(static_cast<float>(cr), 0.f, 255.f));
            }
        }
    }
    return frame;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string workDir = argc > 1 ? argv[1] : "/tmp/bulletcam_test";
    ensureDirectory(workDir);

    ReconstructionOptions options;
    options.stereoImageMaxEdge = 160;
    options.textureImageMaxEdge = 480;
    options.patchMatchIterations = 3;
    options.sourceViewsPerRef = 4;
    options.poissonResolution = 112;
    options.atlasResolution = 1024;
    options.targetTriangles = 0;

    FrameStore store(workDir + "/frames", options);
    ensureDirectory(workDir + "/frames");

    int accepted = 0;
    for (int i = 0; i < 20; ++i) {
        const float azimuth = static_cast<float>(i) * 18.f;
        const float elevation = 10.f + 18.f * std::sin(static_cast<float>(i) * 0.7f);
        RenderedFrame frame = render(azimuth, elevation);
        const float score = store.addFrame(frame.y.data(), frame.u.data(), frame.v.data(),
                                           kImageWidth, kImageHeight, kImageWidth,
                                           kImageWidth / 2, 1, frame.cameraToWorld,
                                           frame.intrinsics, 0.f);
        if (score >= 0.f) ++accepted;
    }
    std::printf("Ingested %d frames\n", accepted);

    Progress progress([](const std::string& stage, float value) {
        std::printf("  [%3.0f%%] %s\n", value * 100.f, stage.c_str());
        std::fflush(stdout);
    });

    const ReconstructionResult result =
        runReconstruction(store, workDir + "/model.obj", workDir + "/model.glb", progress);
    std::printf("success=%d points=%d triangles=%d message=%s\n", result.success ? 1 : 0,
                result.pointCount, result.triangleCount, result.message.c_str());
    if (!result.success) return 1;

    // Ground truth check: every vertex should sit on the sphere we rendered.
    std::vector<float> errors;
    if (FILE* obj = std::fopen((workDir + "/model.obj").c_str(), "r")) {
        char line[256];
        while (std::fgets(line, sizeof(line), obj)) {
            float x, y, z;
            if (std::sscanf(line, "v %f %f %f", &x, &y, &z) == 3)
                errors.push_back(std::fabs(std::sqrt(x * x + y * y + z * z) - kSphereRadius));
        }
        std::fclose(obj);
    }
    if (errors.size() < 1000) {
        std::printf("FAIL: exported mesh has too few vertices\n");
        return 1;
    }
    std::sort(errors.begin(), errors.end());
    const float median = errors[errors.size() / 2];
    std::printf("median radial error = %.2f mm over %zu vertices\n", median * 1000.f, errors.size());
    if (median > 0.003f) {
        std::printf("FAIL: median radial error above the 3 mm tolerance\n");
        return 1;
    }
    std::printf("PASS\n");
    return 0;
}
