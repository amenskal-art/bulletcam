#pragma once
#include <cstdint>
#include <vector>

#include "core/types.h"

namespace bc {

struct Mesh {
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec2> uvs;
    std::vector<uint32_t> indices;

    size_t vertexCount() const { return positions.size(); }
    size_t triangleCount() const { return indices.size() / 3; }
    bool empty() const { return indices.empty(); }
    void clear() {
        positions.clear();
        normals.clear();
        uvs.clear();
        indices.clear();
    }
};

BBox meshBounds(const Mesh& mesh);

// Area-weighted vertex normals.
void computeVertexNormals(Mesh& mesh);

// Keeps only components with at least `minTriangles` faces, and always keeps
// the largest one so a strict threshold can never empty the mesh.
void removeSmallComponents(Mesh& mesh, int minTriangles);

// Taubin lambda/mu smoothing: denoises without the shrinkage of plain Laplacian.
void taubinSmooth(Mesh& mesh, int iterations, float lambda, float mu);

// Vertex clustering decimation down to roughly `targetTriangles`.
void decimate(Mesh& mesh, int targetTriangles);

// Drops unreferenced vertices and degenerate faces.
void compact(Mesh& mesh);

}  // namespace bc
