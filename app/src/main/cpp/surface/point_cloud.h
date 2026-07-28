#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "core/types.h"

namespace bc {

struct PointCloud {
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec3> colors;  // linear 0..1

    size_t size() const { return positions.size(); }
    void reserve(size_t n) {
        positions.reserve(n);
        normals.reserve(n);
        colors.reserve(n);
    }
    void push(const Vec3& p, const Vec3& n, const Vec3& c) {
        positions.push_back(p);
        normals.push_back(n);
        colors.push_back(c);
    }
    void clear() {
        positions.clear();
        normals.clear();
        colors.clear();
    }
};

// Uniform spatial hash. After voxel downsampling the cloud is close to evenly
// spaced, so a hashed grid beats a kd-tree here on both build and query cost.
class SpatialHash {
public:
    SpatialHash(const std::vector<Vec3>& points, float cellSize);
    void knnSearch(const Vec3& query, int k, std::vector<std::pair<float, int>>& out) const;
    void radiusSearch(const Vec3& query, float radius, std::vector<int>& out) const;

private:
    static int64_t key(int x, int y, int z) {
        return (static_cast<int64_t>(x) * 73856093) ^ (static_cast<int64_t>(y) * 19349663) ^
               (static_cast<int64_t>(z) * 83492791);
    }
    void collect(const Vec3& query, int ring, std::vector<int>& out) const;

    const std::vector<Vec3>& points_;
    float cell_ = 1.f;
    std::unordered_map<int64_t, std::vector<int>> cells_;
};

BBox computeBounds(const PointCloud& cloud);

// Averages positions, normals and colours inside each voxel.
void voxelDownsample(PointCloud& cloud, float voxelSize);

// Drops points whose mean distance to their k nearest neighbours is more than
// `stdDevMultiplier` standard deviations above the global mean.
void removeStatisticalOutliers(PointCloud& cloud, int neighbours, float stdDevMultiplier);

// Re-fits normals by local PCA, keeping the orientation of the incoming normals
// (which came from the stereo stage and therefore already face the cameras).
void refineNormals(PointCloud& cloud, int neighbours);

}  // namespace bc
