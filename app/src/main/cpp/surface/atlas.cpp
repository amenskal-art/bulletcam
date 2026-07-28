#include "surface/atlas.h"

#include <algorithm>
#include <array>
#include <unordered_map>
#include <vector>

#include "core/log.h"

#if BULLETCAM_HAVE_XATLAS
#include "xatlas.h"
#endif

namespace bc {
namespace {

constexpr float kChartNormalCosine = 0.55f;   // ~57 degrees from the seed normal
constexpr int kMaxChartTriangles = 20000;
constexpr float kChartPadding = 2.f;

struct Chart {
    std::vector<uint32_t> triangles;
    Vec3 normal;
    Vec3 axisU, axisV;
    Vec2 boxMin;
    float width = 0.f;
    float height = 0.f;
    Vec2 placement;
};

Vec3 faceNormal(const Mesh& mesh, size_t triangle) {
    const Vec3& a = mesh.positions[mesh.indices[triangle * 3 + 0]];
    const Vec3& b = mesh.positions[mesh.indices[triangle * 3 + 1]];
    const Vec3& c = mesh.positions[mesh.indices[triangle * 3 + 2]];
    const Vec3 n = (b - a).cross(c - a);
    return n.squaredNorm() > 1e-24f ? n.normalized() : Vec3{0.f, 1.f, 0.f};
}

// Triangle adjacency across shared edges.
std::vector<std::array<int, 3>> buildAdjacency(const Mesh& mesh) {
    const size_t triangleCount = mesh.triangleCount();
    std::vector<std::array<int, 3>> adjacency(triangleCount, {{-1, -1, -1}});
    std::unordered_map<uint64_t, std::pair<int, int>> edges;
    edges.reserve(triangleCount * 3);

    for (size_t t = 0; t < triangleCount; ++t) {
        for (int e = 0; e < 3; ++e) {
            uint32_t a = mesh.indices[t * 3 + static_cast<size_t>(e)];
            uint32_t b = mesh.indices[t * 3 + static_cast<size_t>((e + 1) % 3)];
            if (a > b) std::swap(a, b);
            const uint64_t key = (static_cast<uint64_t>(a) << 32) | b;
            auto it = edges.find(key);
            if (it == edges.end()) {
                edges.emplace(key, std::make_pair(static_cast<int>(t), e));
            } else {
                const int otherTriangle = it->second.first;
                const int otherEdge = it->second.second;
                adjacency[t][static_cast<size_t>(e)] = otherTriangle;
                adjacency[static_cast<size_t>(otherTriangle)][static_cast<size_t>(otherEdge)] =
                    static_cast<int>(t);
            }
        }
    }
    return adjacency;
}

// Region-grows near-planar charts, projects each to its own plane and shelf
// packs them. Charts rather than individual triangles keep the padding cost
// proportional to the number of seams instead of the number of faces.
bool chartPack(Mesh& mesh, int atlasResolution) {
    const size_t triangleCount = mesh.triangleCount();
    if (triangleCount == 0) return false;

    const std::vector<std::array<int, 3>> adjacency = buildAdjacency(mesh);
    std::vector<int> chartOf(triangleCount, -1);
    std::vector<Chart> charts;
    std::vector<uint32_t> queue;

    for (size_t seed = 0; seed < triangleCount; ++seed) {
        if (chartOf[seed] >= 0) continue;
        Chart chart;
        chart.normal = faceNormal(mesh, seed);
        const int chartId = static_cast<int>(charts.size());

        queue.clear();
        queue.push_back(static_cast<uint32_t>(seed));
        chartOf[seed] = chartId;
        for (size_t head = 0; head < queue.size(); ++head) {
            const uint32_t t = queue[head];
            chart.triangles.push_back(t);
            if (static_cast<int>(chart.triangles.size()) >= kMaxChartTriangles) break;
            for (const int neighbour : adjacency[t]) {
                if (neighbour < 0 || chartOf[static_cast<size_t>(neighbour)] >= 0) continue;
                if (faceNormal(mesh, static_cast<size_t>(neighbour)).dot(chart.normal) <
                    kChartNormalCosine)
                    continue;
                chartOf[static_cast<size_t>(neighbour)] = chartId;
                queue.push_back(static_cast<uint32_t>(neighbour));
            }
        }
        // Triangles queued past the size cap stay unassigned for the next chart.
        for (size_t i = chart.triangles.size(); i < queue.size(); ++i)
            chartOf[queue[i]] = -1;

        Vec3 helper = std::fabs(chart.normal.x) < 0.9f ? Vec3{1.f, 0.f, 0.f} : Vec3{0.f, 1.f, 0.f};
        chart.axisU = chart.normal.cross(helper).normalized();
        chart.axisV = chart.normal.cross(chart.axisU);
        charts.push_back(std::move(chart));
    }

    // Measure each chart in its own plane.
    double totalArea = 0.0;
    for (Chart& chart : charts) {
        float minU = 1e30f, maxU = -1e30f, minV = 1e30f, maxV = -1e30f;
        for (const uint32_t t : chart.triangles) {
            for (int corner = 0; corner < 3; ++corner) {
                const Vec3& p = mesh.positions[mesh.indices[static_cast<size_t>(t) * 3 +
                                                            static_cast<size_t>(corner)]];
                const float u = p.dot(chart.axisU);
                const float v = p.dot(chart.axisV);
                minU = std::min(minU, u); maxU = std::max(maxU, u);
                minV = std::min(minV, v); maxV = std::max(maxV, v);
            }
        }
        chart.boxMin = {minU, minV};
        chart.width = std::max(1e-6f, maxU - minU);
        chart.height = std::max(1e-6f, maxV - minV);
        totalArea += static_cast<double>(chart.width) * chart.height;
    }
    if (totalArea <= 0.0) return false;

    const float atlas = static_cast<float>(atlasResolution);
    float scale = std::sqrt(0.7f * atlas * atlas / static_cast<float>(totalArea));

    std::vector<size_t> order(charts.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&](size_t l, size_t r) { return charts[l].height > charts[r].height; });

    bool packed = false;
    for (int attempt = 0; attempt < 40 && !packed; ++attempt) {
        float cursorX = kChartPadding, cursorY = kChartPadding, shelfHeight = 0.f;
        packed = true;
        for (const size_t i : order) {
            const float w = charts[i].width * scale;
            const float h = charts[i].height * scale;
            if (w + 2.f * kChartPadding > atlas || h + 2.f * kChartPadding > atlas) {
                packed = false;
                break;
            }
            if (cursorX + w + kChartPadding > atlas) {
                cursorX = kChartPadding;
                cursorY += shelfHeight + kChartPadding;
                shelfHeight = 0.f;
            }
            if (cursorY + h + kChartPadding > atlas) {
                packed = false;
                break;
            }
            charts[i].placement = {cursorX, cursorY};
            cursorX += w + kChartPadding;
            shelfHeight = std::max(shelfHeight, h);
        }
        if (!packed) scale *= 0.85f;
    }
    if (!packed) {
        BC_LOGE("Atlas packing failed even at minimum texel density");
        return false;
    }

    // Emit one vertex per (chart, original vertex) pair so charts never share UVs.
    Mesh out;
    out.positions.reserve(mesh.positions.size() * 3 / 2);
    out.normals.reserve(mesh.positions.size() * 3 / 2);
    out.uvs.reserve(mesh.positions.size() * 3 / 2);
    out.indices.reserve(mesh.indices.size());
    const bool hasNormals = mesh.normals.size() == mesh.positions.size();
    std::unordered_map<uint64_t, uint32_t> emitted;
    emitted.reserve(mesh.positions.size() * 2);

    for (size_t c = 0; c < charts.size(); ++c) {
        const Chart& chart = charts[c];
        for (const uint32_t t : chart.triangles) {
            for (int corner = 0; corner < 3; ++corner) {
                const uint32_t source =
                    mesh.indices[static_cast<size_t>(t) * 3 + static_cast<size_t>(corner)];
                const uint64_t key = (static_cast<uint64_t>(c) << 32) | source;
                auto it = emitted.find(key);
                if (it != emitted.end()) {
                    out.indices.push_back(it->second);
                    continue;
                }
                const Vec3& p = mesh.positions[source];
                const float u = (p.dot(chart.axisU) - chart.boxMin.x) * scale + chart.placement.x;
                const float v = (p.dot(chart.axisV) - chart.boxMin.y) * scale + chart.placement.y;
                const uint32_t index = static_cast<uint32_t>(out.positions.size());
                out.positions.push_back(p);
                out.normals.push_back(hasNormals ? mesh.normals[source] : chart.normal);
                out.uvs.push_back({u / atlas, v / atlas});
                emitted.emplace(key, index);
                out.indices.push_back(index);
            }
        }
    }

    BC_LOGI("Atlas: %zu charts, %zu vertices, texel scale %.1f/m", charts.size(),
            out.positions.size(), static_cast<double>(scale));
    mesh = std::move(out);
    return true;
}

}  // namespace

bool buildAtlas(Mesh& mesh, int atlasResolution) {
    if (mesh.empty()) return false;

#if BULLETCAM_HAVE_XATLAS
    xatlas::Atlas* atlas = xatlas::Create();
    xatlas::MeshDecl decl;
    decl.vertexCount = static_cast<uint32_t>(mesh.positions.size());
    decl.vertexPositionData = mesh.positions.data();
    decl.vertexPositionStride = sizeof(Vec3);
    if (mesh.normals.size() == mesh.positions.size()) {
        decl.vertexNormalData = mesh.normals.data();
        decl.vertexNormalStride = sizeof(Vec3);
    }
    decl.indexCount = static_cast<uint32_t>(mesh.indices.size());
    decl.indexData = mesh.indices.data();
    decl.indexFormat = xatlas::IndexFormat::UInt32;

    if (xatlas::AddMesh(atlas, decl) != xatlas::AddMeshError::Success) {
        xatlas::Destroy(atlas);
        BC_LOGW("xatlas rejected the mesh, falling back to shelf packing");
        return chartPack(mesh, atlasResolution);
    }

    xatlas::ChartOptions chartOptions;
    xatlas::PackOptions packOptions;
    packOptions.resolution = static_cast<uint32_t>(atlasResolution);
    packOptions.padding = 2;
    packOptions.bruteForce = false;
    xatlas::Generate(atlas, chartOptions, packOptions);

    if (atlas->meshCount == 0 || atlas->width == 0 || atlas->height == 0) {
        xatlas::Destroy(atlas);
        return chartPack(mesh, atlasResolution);
    }

    const xatlas::Mesh& result = atlas->meshes[0];
    Mesh out;
    out.positions.resize(result.vertexCount);
    out.normals.resize(result.vertexCount);
    out.uvs.resize(result.vertexCount);
    const bool hasNormals = mesh.normals.size() == mesh.positions.size();
    for (uint32_t i = 0; i < result.vertexCount; ++i) {
        const xatlas::Vertex& v = result.vertexArray[i];
        out.positions[i] = mesh.positions[v.xref];
        out.normals[i] = hasNormals ? mesh.normals[v.xref] : Vec3{0.f, 1.f, 0.f};
        out.uvs[i] = {v.uv[0] / static_cast<float>(atlas->width),
                      v.uv[1] / static_cast<float>(atlas->height)};
    }
    out.indices.assign(result.indexArray, result.indexArray + result.indexCount);
    xatlas::Destroy(atlas);
    mesh = std::move(out);
    return true;
#else
    return chartPack(mesh, atlasResolution);
#endif
}

}  // namespace bc
