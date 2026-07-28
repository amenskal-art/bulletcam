#include "surface/marching_tets.h"

#include <unordered_map>

namespace bc {
namespace {

// Cube corner offsets, indexed by the bit pattern (x, y, z).
constexpr int kCornerOffset[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                                     {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};

// Six positively oriented tetrahedra covering the cube, all sharing edge 0-6.
constexpr int kTetrahedra[6][4] = {{0, 1, 2, 6}, {0, 2, 3, 6}, {0, 3, 7, 6},
                                   {0, 7, 4, 6}, {0, 4, 5, 6}, {0, 5, 1, 6}};

struct EdgeVertex {
    int a, b;  // local tetra corner indices
};

}  // namespace

Mesh extractIsosurface(const ScalarGrid& grid, float iso, const std::vector<uint8_t>& vertexMask) {
    Mesh mesh;
    if (grid.nx < 2 || grid.ny < 2 || grid.nz < 2) return mesh;

    const bool useMask = vertexMask.size() == grid.values.size();
    std::unordered_map<uint64_t, uint32_t> edgeVertices;
    edgeVertices.reserve(1u << 16);

    auto globalIndex = [&](int x, int y, int z) -> uint64_t {
        return static_cast<uint64_t>(grid.index(x, y, z));
    };

    // Creates (or reuses) the interpolated vertex on the grid edge p-q.
    auto vertexOnEdge = [&](int px, int py, int pz, float fp, int qx, int qy, int qz,
                            float fq) -> uint32_t {
        uint64_t a = globalIndex(px, py, pz);
        uint64_t b = globalIndex(qx, qy, qz);
        if (a > b) {
            std::swap(a, b);
            std::swap(fp, fq);
            std::swap(px, qx);
            std::swap(py, qy);
            std::swap(pz, qz);
        }
        const uint64_t key = a * 0x100000000ull + b;
        auto it = edgeVertices.find(key);
        if (it != edgeVertices.end()) return it->second;

        const float denom = fq - fp;
        const float t = std::fabs(denom) < 1e-12f ? 0.5f : clampf((iso - fp) / denom, 0.f, 1.f);
        const Vec3 pa = grid.position(px, py, pz);
        const Vec3 pb = grid.position(qx, qy, qz);
        const uint32_t index = static_cast<uint32_t>(mesh.positions.size());
        mesh.positions.push_back(pa + (pb - pa) * t);
        edgeVertices.emplace(key, index);
        return index;
    };

    auto emit = [&](uint32_t a, uint32_t b, uint32_t c) {
        if (a == b || b == c || a == c) return;
        const Vec3& pa = mesh.positions[a];
        const Vec3& pb = mesh.positions[b];
        const Vec3& pc = mesh.positions[c];
        const Vec3 faceNormal = (pb - pa).cross(pc - pa);
        // Orient every face outward using the field gradient at its centroid.
        // chi rises from inside to outside, so the gradient is the outward axis.
        const Vec3 centroid = (pa + pb + pc) / 3.f;
        const Vec3 g = (centroid - grid.origin) / grid.h;
        const int gx = std::min(grid.nx - 1, std::max(0, static_cast<int>(g.x)));
        const int gy = std::min(grid.ny - 1, std::max(0, static_cast<int>(g.y)));
        const int gz = std::min(grid.nz - 1, std::max(0, static_cast<int>(g.z)));
        const Vec3 gradient = grid.gradient(gx, gy, gz);
        if (faceNormal.dot(gradient) < 0.f) {
            mesh.indices.push_back(a);
            mesh.indices.push_back(c);
            mesh.indices.push_back(b);
        } else {
            mesh.indices.push_back(a);
            mesh.indices.push_back(b);
            mesh.indices.push_back(c);
        }
    };

    int corner[8][3];
    float value[8];

    for (int z = 0; z + 1 < grid.nz; ++z) {
        for (int y = 0; y + 1 < grid.ny; ++y) {
            for (int x = 0; x + 1 < grid.nx; ++x) {
                bool masked = false;
                for (int c = 0; c < 8; ++c) {
                    corner[c][0] = x + kCornerOffset[c][0];
                    corner[c][1] = y + kCornerOffset[c][1];
                    corner[c][2] = z + kCornerOffset[c][2];
                    const size_t idx = grid.index(corner[c][0], corner[c][1], corner[c][2]);
                    value[c] = grid.values[idx];
                    if (useMask && !vertexMask[idx]) masked = true;
                }
                if (masked) continue;

                // Skip cubes entirely on one side of the isosurface.
                bool anyBelow = false, anyAbove = false;
                for (int c = 0; c < 8; ++c) {
                    if (value[c] < iso) anyBelow = true;
                    else anyAbove = true;
                }
                if (!anyBelow || !anyAbove) continue;

                for (const auto& tet : kTetrahedra) {
                    const float f[4] = {value[tet[0]], value[tet[1]], value[tet[2]], value[tet[3]]};
                    const int* v[4] = {corner[tet[0]], corner[tet[1]], corner[tet[2]],
                                       corner[tet[3]]};
                    int code = 0;
                    for (int i = 0; i < 4; ++i)
                        if (f[i] < iso) code |= 1 << i;
                    if (code == 0 || code == 15) continue;

                    auto edge = [&](int i, int j) {
                        return vertexOnEdge(v[i][0], v[i][1], v[i][2], f[i], v[j][0], v[j][1],
                                            v[j][2], f[j]);
                    };

                    switch (code) {
                        case 0x01:
                        case 0x0E:
                            emit(edge(0, 1), edge(0, 2), edge(0, 3));
                            break;
                        case 0x02:
                        case 0x0D:
                            emit(edge(1, 0), edge(1, 3), edge(1, 2));
                            break;
                        case 0x04:
                        case 0x0B:
                            emit(edge(2, 0), edge(2, 1), edge(2, 3));
                            break;
                        case 0x08:
                        case 0x07:
                            emit(edge(3, 0), edge(3, 2), edge(3, 1));
                            break;
                        case 0x03:
                        case 0x0C: {
                            const uint32_t e02 = edge(0, 2), e03 = edge(0, 3);
                            const uint32_t e13 = edge(1, 3), e12 = edge(1, 2);
                            emit(e02, e03, e13);
                            emit(e02, e13, e12);
                            break;
                        }
                        case 0x05:
                        case 0x0A: {
                            const uint32_t e01 = edge(0, 1), e12 = edge(1, 2);
                            const uint32_t e23 = edge(2, 3), e03 = edge(0, 3);
                            emit(e01, e12, e23);
                            emit(e01, e23, e03);
                            break;
                        }
                        case 0x09:
                        case 0x06: {
                            const uint32_t e01 = edge(0, 1), e13 = edge(1, 3);
                            const uint32_t e23 = edge(2, 3), e02 = edge(0, 2);
                            emit(e01, e13, e23);
                            emit(e01, e23, e02);
                            break;
                        }
                        default:
                            break;
                    }
                }
            }
        }
    }

    computeVertexNormals(mesh);
    return mesh;
}

}  // namespace bc
