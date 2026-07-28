#include "surface/point_cloud.h"

#include <algorithm>

#include "core/parallel.h"

namespace bc {

SpatialHash::SpatialHash(const std::vector<Vec3>& points, float cellSize)
    : points_(points), cell_(cellSize > 1e-9f ? cellSize : 1e-9f) {
    cells_.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        const Vec3& p = points[i];
        const int x = static_cast<int>(std::floor(p.x / cell_));
        const int y = static_cast<int>(std::floor(p.y / cell_));
        const int z = static_cast<int>(std::floor(p.z / cell_));
        cells_[key(x, y, z)].push_back(static_cast<int>(i));
    }
}

void SpatialHash::collect(const Vec3& query, int ring, std::vector<int>& out) const {
    const int cx = static_cast<int>(std::floor(query.x / cell_));
    const int cy = static_cast<int>(std::floor(query.y / cell_));
    const int cz = static_cast<int>(std::floor(query.z / cell_));
    for (int dz = -ring; dz <= ring; ++dz) {
        for (int dy = -ring; dy <= ring; ++dy) {
            for (int dx = -ring; dx <= ring; ++dx) {
                // Only visit the shell that was added by growing the ring.
                if (ring > 0 && std::abs(dx) != ring && std::abs(dy) != ring &&
                    std::abs(dz) != ring)
                    continue;
                auto it = cells_.find(key(cx + dx, cy + dy, cz + dz));
                if (it == cells_.end()) continue;
                out.insert(out.end(), it->second.begin(), it->second.end());
            }
        }
    }
}

void SpatialHash::knnSearch(const Vec3& query, int k, std::vector<std::pair<float, int>>& out) const {
    out.clear();
    std::vector<int> candidates;
    for (int ring = 0; ring <= 6; ++ring) {
        collect(query, ring, candidates);
        if (static_cast<int>(candidates.size()) >= k + 1 && ring >= 1) break;
    }
    if (candidates.empty()) return;
    out.reserve(candidates.size());
    for (const int idx : candidates)
        out.emplace_back((points_[static_cast<size_t>(idx)] - query).squaredNorm(), idx);
    const size_t keep = std::min(out.size(), static_cast<size_t>(k));
    std::partial_sort(out.begin(), out.begin() + static_cast<long>(keep), out.end());
    out.resize(keep);
}

void SpatialHash::radiusSearch(const Vec3& query, float radius, std::vector<int>& out) const {
    out.clear();
    const int ring = std::max(1, static_cast<int>(std::ceil(radius / cell_)));
    std::vector<int> candidates;
    for (int r = 0; r <= ring; ++r) collect(query, r, candidates);
    const float r2 = radius * radius;
    for (const int idx : candidates)
        if ((points_[static_cast<size_t>(idx)] - query).squaredNorm() <= r2) out.push_back(idx);
}

BBox computeBounds(const PointCloud& cloud) {
    BBox box;
    for (const Vec3& p : cloud.positions) box.extend(p);
    return box;
}

void voxelDownsample(PointCloud& cloud, float voxelSize) {
    if (cloud.size() == 0 || voxelSize <= 0.f) return;

    struct Accum {
        Vec3 position, normal, color;
        int count = 0;
    };
    std::unordered_map<int64_t, Accum> grid;
    grid.reserve(cloud.size());

    for (size_t i = 0; i < cloud.size(); ++i) {
        const Vec3& p = cloud.positions[i];
        const int64_t x = static_cast<int64_t>(std::floor(p.x / voxelSize));
        const int64_t y = static_cast<int64_t>(std::floor(p.y / voxelSize));
        const int64_t z = static_cast<int64_t>(std::floor(p.z / voxelSize));
        const int64_t k = (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
        Accum& a = grid[k];
        a.position += p;
        a.normal += cloud.normals[i];
        a.color += cloud.colors[i];
        a.count += 1;
    }

    PointCloud out;
    out.reserve(grid.size());
    for (const auto& entry : grid) {
        const Accum& a = entry.second;
        const float inv = 1.f / static_cast<float>(a.count);
        Vec3 n = a.normal * inv;
        if (n.squaredNorm() < 1e-8f) continue;
        out.push(a.position * inv, n.normalized(), a.color * inv);
    }
    cloud = std::move(out);
}

void removeStatisticalOutliers(PointCloud& cloud, int neighbours, float stdDevMultiplier) {
    const int n = static_cast<int>(cloud.size());
    if (n < neighbours * 4) return;

    BBox box = computeBounds(cloud);
    const float cell = std::max(1e-6f, box.maxExtent() / 96.f);
    SpatialHash hash(cloud.positions, cell);

    std::vector<float> meanDist(static_cast<size_t>(n), 0.f);
    parallelBlocks(n, [&](int begin, int end) {
        std::vector<std::pair<float, int>> knn;
        for (int i = begin; i < end; ++i) {
            hash.knnSearch(cloud.positions[static_cast<size_t>(i)], neighbours + 1, knn);
            float sum = 0.f;
            int count = 0;
            for (const auto& e : knn) {
                if (e.second == i) continue;
                sum += std::sqrt(e.first);
                ++count;
            }
            meanDist[static_cast<size_t>(i)] = count > 0 ? sum / static_cast<float>(count) : 0.f;
        }
    });

    double sum = 0.0, sumSq = 0.0;
    int valid = 0;
    for (const float d : meanDist) {
        if (d <= 0.f) continue;
        sum += d;
        sumSq += static_cast<double>(d) * d;
        ++valid;
    }
    if (valid < 8) return;
    const double mean = sum / valid;
    const double variance = std::max(0.0, sumSq / valid - mean * mean);
    const float threshold = static_cast<float>(mean + stdDevMultiplier * std::sqrt(variance));

    PointCloud out;
    out.reserve(cloud.size());
    for (int i = 0; i < n; ++i) {
        const float d = meanDist[static_cast<size_t>(i)];
        if (d <= 0.f || d > threshold) continue;
        out.push(cloud.positions[static_cast<size_t>(i)], cloud.normals[static_cast<size_t>(i)],
                 cloud.colors[static_cast<size_t>(i)]);
    }
    cloud = std::move(out);
}

void refineNormals(PointCloud& cloud, int neighbours) {
    const int n = static_cast<int>(cloud.size());
    if (n < neighbours * 2) return;

    BBox box = computeBounds(cloud);
    const float cell = std::max(1e-6f, box.maxExtent() / 96.f);
    SpatialHash hash(cloud.positions, cell);
    std::vector<Vec3> refined(static_cast<size_t>(n));

    parallelBlocks(n, [&](int begin, int end) {
        std::vector<std::pair<float, int>> knn;
        for (int i = begin; i < end; ++i) {
            const Vec3& p = cloud.positions[static_cast<size_t>(i)];
            hash.knnSearch(p, neighbours, knn);
            if (knn.size() < 4) {
                refined[static_cast<size_t>(i)] = cloud.normals[static_cast<size_t>(i)];
                continue;
            }
            Vec3 mean{0.f, 0.f, 0.f};
            for (const auto& e : knn) mean += cloud.positions[static_cast<size_t>(e.second)];
            mean = mean / static_cast<float>(knn.size());

            Mat3 cov = Mat3::zero();
            for (const auto& e : knn) {
                const Vec3 d = cloud.positions[static_cast<size_t>(e.second)] - mean;
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c) cov(r, c) += d[r] * d[c];
            }
            Vec3 eigenvalues;
            Mat3 eigenvectors;
            symmetricEigen(cov, eigenvalues, eigenvectors);
            Vec3 normal{eigenvectors(0, 0), eigenvectors(0, 1), eigenvectors(0, 2)};
            // Keep the stereo-derived orientation: those normals already face the
            // cameras that observed the surface.
            if (normal.dot(cloud.normals[static_cast<size_t>(i)]) < 0.f) normal = -normal;
            refined[static_cast<size_t>(i)] = normal.normalized();
        }
    });
    cloud.normals = std::move(refined);
}

}  // namespace bc
