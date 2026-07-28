#include "surface/mesh.h"

#include <algorithm>
#include <numeric>
#include <unordered_map>

namespace bc {
namespace {

struct UnionFind {
    std::vector<int> parent;
    explicit UnionFind(int n) : parent(static_cast<size_t>(n)) {
        std::iota(parent.begin(), parent.end(), 0);
    }
    int find(int a) {
        while (parent[static_cast<size_t>(a)] != a) {
            parent[static_cast<size_t>(a)] = parent[static_cast<size_t>(parent[static_cast<size_t>(a)])];
            a = parent[static_cast<size_t>(a)];
        }
        return a;
    }
    void unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a != b) parent[static_cast<size_t>(b)] = a;
    }
};

}  // namespace

BBox meshBounds(const Mesh& mesh) {
    BBox box;
    for (const Vec3& p : mesh.positions) box.extend(p);
    return box;
}

void computeVertexNormals(Mesh& mesh) {
    mesh.normals.assign(mesh.positions.size(), Vec3{0.f, 0.f, 0.f});
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const uint32_t a = mesh.indices[i], b = mesh.indices[i + 1], c = mesh.indices[i + 2];
        const Vec3& pa = mesh.positions[a];
        const Vec3& pb = mesh.positions[b];
        const Vec3& pc = mesh.positions[c];
        // Un-normalised cross product weights each face by twice its area.
        const Vec3 faceNormal = (pb - pa).cross(pc - pa);
        mesh.normals[a] += faceNormal;
        mesh.normals[b] += faceNormal;
        mesh.normals[c] += faceNormal;
    }
    for (Vec3& n : mesh.normals) {
        if (n.squaredNorm() < 1e-20f) n = Vec3{0.f, 1.f, 0.f};
        else n = n.normalized();
    }
}

void removeSmallComponents(Mesh& mesh, int minTriangles) {
    if (mesh.empty()) return;
    UnionFind uf(static_cast<int>(mesh.positions.size()));
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        uf.unite(static_cast<int>(mesh.indices[i]), static_cast<int>(mesh.indices[i + 1]));
        uf.unite(static_cast<int>(mesh.indices[i]), static_cast<int>(mesh.indices[i + 2]));
    }

    std::unordered_map<int, int> triangleCount;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3)
        triangleCount[uf.find(static_cast<int>(mesh.indices[i]))] += 1;

    int largestRoot = -1, largestCount = 0;
    for (const auto& entry : triangleCount) {
        if (entry.second > largestCount) {
            largestCount = entry.second;
            largestRoot = entry.first;
        }
    }

    std::vector<uint32_t> kept;
    kept.reserve(mesh.indices.size());
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const int root = uf.find(static_cast<int>(mesh.indices[i]));
        if (root != largestRoot && triangleCount[root] < minTriangles) continue;
        kept.push_back(mesh.indices[i]);
        kept.push_back(mesh.indices[i + 1]);
        kept.push_back(mesh.indices[i + 2]);
    }
    mesh.indices = std::move(kept);
    compact(mesh);
}

void taubinSmooth(Mesh& mesh, int iterations, float lambda, float mu) {
    if (mesh.empty() || iterations <= 0) return;
    const size_t vertexCount = mesh.positions.size();

    std::vector<std::vector<uint32_t>> adjacency(vertexCount);
    {
        std::vector<std::pair<uint32_t, uint32_t>> edges;
        edges.reserve(mesh.indices.size() * 2);
        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            const uint32_t a = mesh.indices[i], b = mesh.indices[i + 1], c = mesh.indices[i + 2];
            edges.emplace_back(a, b);
            edges.emplace_back(b, a);
            edges.emplace_back(b, c);
            edges.emplace_back(c, b);
            edges.emplace_back(c, a);
            edges.emplace_back(a, c);
        }
        std::sort(edges.begin(), edges.end());
        edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
        for (const auto& e : edges) adjacency[e.first].push_back(e.second);
    }

    std::vector<Vec3> buffer(vertexCount);
    auto step = [&](float factor) {
        for (size_t i = 0; i < vertexCount; ++i) {
            const auto& neighbours = adjacency[i];
            if (neighbours.empty()) {
                buffer[i] = mesh.positions[i];
                continue;
            }
            Vec3 mean{0.f, 0.f, 0.f};
            for (const uint32_t j : neighbours) mean += mesh.positions[j];
            mean = mean / static_cast<float>(neighbours.size());
            buffer[i] = mesh.positions[i] + (mean - mesh.positions[i]) * factor;
        }
        mesh.positions.swap(buffer);
    };

    for (int i = 0; i < iterations; ++i) {
        step(lambda);
        step(mu);
    }
    computeVertexNormals(mesh);
}

void decimate(Mesh& mesh, int targetTriangles) {
    if (mesh.empty() || targetTriangles <= 0) return;
    if (static_cast<int>(mesh.triangleCount()) <= targetTriangles) return;
    if (mesh.normals.size() != mesh.positions.size()) computeVertexNormals(mesh);

    const BBox box = meshBounds(mesh);
    const float extent = std::max(1e-6f, box.maxExtent());
    // Vertices of a closed mesh sit on a surface, so the cluster count grows
    // with the square of the cell density. Solve for the cell size that lands
    // near the requested budget instead of guessing once.
    const int targetVertices = std::max(16, targetTriangles / 2);
    float cell = extent / std::sqrt(static_cast<float>(targetVertices));

    auto cellKey = [](const Vec3& p, float size) {
        const int64_t x = static_cast<int64_t>(std::floor(p.x / size));
        const int64_t y = static_cast<int64_t>(std::floor(p.y / size));
        const int64_t z = static_cast<int64_t>(std::floor(p.z / size));
        return (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
    };

    for (int attempt = 0; attempt < 6; ++attempt) {
        std::unordered_map<int64_t, uint32_t> probe;
        probe.reserve(mesh.positions.size());
        for (const Vec3& p : mesh.positions) probe.emplace(cellKey(p, cell), 0u);
        const float achieved = static_cast<float>(probe.size());
        const float ratio = achieved / static_cast<float>(targetVertices);
        if (ratio > 0.85f && ratio < 1.25f) break;
        cell *= std::sqrt(clampf(ratio, 0.25f, 4.f));
    }

    std::unordered_map<int64_t, uint32_t> clusterIndex;
    std::vector<Vec3> accumPos;
    std::vector<Vec3> accumNormal;
    std::vector<int> accumCount;
    std::vector<uint32_t> remap(mesh.positions.size());

    for (size_t i = 0; i < mesh.positions.size(); ++i) {
        const int64_t key = cellKey(mesh.positions[i], cell);
        auto it = clusterIndex.find(key);
        if (it == clusterIndex.end()) {
            const uint32_t id = static_cast<uint32_t>(accumPos.size());
            clusterIndex.emplace(key, id);
            accumPos.push_back(mesh.positions[i]);
            accumNormal.push_back(mesh.normals[i]);
            accumCount.push_back(1);
            remap[i] = id;
        } else {
            accumPos[it->second] += mesh.positions[i];
            accumNormal[it->second] += mesh.normals[i];
            accumCount[it->second] += 1;
            remap[i] = it->second;
        }
    }

    for (size_t i = 0; i < accumPos.size(); ++i) {
        accumPos[i] = accumPos[i] / static_cast<float>(accumCount[i]);
        accumNormal[i] = accumNormal[i].squaredNorm() > 1e-20f ? accumNormal[i].normalized()
                                                              : Vec3{0.f, 1.f, 0.f};
    }

    std::vector<uint32_t> indices;
    indices.reserve(mesh.indices.size());
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        uint32_t a = remap[mesh.indices[i]];
        uint32_t b = remap[mesh.indices[i + 1]];
        uint32_t c = remap[mesh.indices[i + 2]];
        if (a == b || b == c || a == c) continue;
        // Clustering can fold a triangle over on itself. The inherited normals
        // still carry the correct side, so re-derive the winding from them.
        const Vec3 faceNormal = (accumPos[b] - accumPos[a]).cross(accumPos[c] - accumPos[a]);
        const Vec3 reference = accumNormal[a] + accumNormal[b] + accumNormal[c];
        if (faceNormal.dot(reference) < 0.f) std::swap(b, c);
        indices.push_back(a);
        indices.push_back(b);
        indices.push_back(c);
    }

    mesh.positions = std::move(accumPos);
    mesh.normals = std::move(accumNormal);
    mesh.uvs.clear();
    mesh.indices = std::move(indices);
    compact(mesh);
}

void compact(Mesh& mesh) {
    std::vector<uint32_t> remap(mesh.positions.size(), 0xFFFFFFFFu);
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec2> uvs;
    std::vector<uint32_t> indices;
    indices.reserve(mesh.indices.size());

    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const uint32_t tri[3] = {mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]};
        if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2]) continue;
        for (const uint32_t v : tri) {
            if (remap[v] == 0xFFFFFFFFu) {
                remap[v] = static_cast<uint32_t>(positions.size());
                positions.push_back(mesh.positions[v]);
                if (!mesh.normals.empty()) normals.push_back(mesh.normals[v]);
                if (!mesh.uvs.empty()) uvs.push_back(mesh.uvs[v]);
            }
            indices.push_back(remap[v]);
        }
    }

    mesh.positions = std::move(positions);
    mesh.normals = std::move(normals);
    mesh.uvs = std::move(uvs);
    mesh.indices = std::move(indices);
}

}  // namespace bc
