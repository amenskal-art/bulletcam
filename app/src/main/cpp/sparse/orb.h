#pragma once
#include <array>
#include <vector>

#include "core/image.h"
#include "core/types.h"

namespace bc {

struct Keypoint {
    float x = 0.f, y = 0.f;
    float angle = 0.f;   // radians, from the intensity centroid
    float score = 0.f;   // FAST corner strength
};

using Descriptor = std::array<uint64_t, 4>;  // 256-bit rBRIEF

struct FeatureSet {
    std::vector<Keypoint> keypoints;
    std::vector<Descriptor> descriptors;
    size_t size() const { return keypoints.size(); }
};

// FAST-9 detection with grid-based spatial distribution, intensity-centroid
// orientation and steered BRIEF description - the ORB recipe used by OpenMVG
// and COLMAP for their binary descriptor front end.
FeatureSet detectAndDescribe(const ImageU8& gray, int maxFeatures, int fastThreshold);

inline int hammingDistance(const Descriptor& a, const Descriptor& b) {
    return __builtin_popcountll(a[0] ^ b[0]) + __builtin_popcountll(a[1] ^ b[1]) +
           __builtin_popcountll(a[2] ^ b[2]) + __builtin_popcountll(a[3] ^ b[3]);
}

}  // namespace bc
