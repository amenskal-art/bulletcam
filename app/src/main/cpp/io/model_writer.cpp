#include "io/model_writer.h"

#include <cstdio>
#include <cstring>
#include <sstream>
#include <vector>

#include "core/log.h"
#include "io/png.h"

namespace bc {
namespace {

std::string stripExtension(const std::string& path) {
    const size_t dot = path.find_last_of('.');
    const size_t slash = path.find_last_of('/');
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return path;
    return path.substr(0, dot);
}

std::string baseName(const std::string& path) {
    const size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

void appendLittleEndian32(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>(value & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
}

void padTo4(std::vector<uint8_t>& out, uint8_t filler) {
    while (out.size() % 4 != 0) out.push_back(filler);
}

}  // namespace

bool writeObj(const std::string& objPath, const Mesh& mesh, const Texture& texture) {
    if (mesh.empty()) return false;
    const std::string stem = stripExtension(objPath);
    const std::string mtlPath = stem + ".mtl";
    const std::string pngPath = stem + ".png";
    const bool hasTexture = !texture.empty() && mesh.uvs.size() == mesh.positions.size();

    if (hasTexture && !writePng(pngPath, texture)) {
        BC_LOGE("Failed to write %s", pngPath.c_str());
        return false;
    }

    if (hasTexture) {
        FILE* mtl = std::fopen(mtlPath.c_str(), "w");
        if (!mtl) return false;
        std::fprintf(mtl,
                     "newmtl bulletcam\n"
                     "Ka 1.000 1.000 1.000\n"
                     "Kd 1.000 1.000 1.000\n"
                     "Ks 0.000 0.000 0.000\n"
                     "d 1.0\n"
                     "illum 2\n"
                     "map_Kd %s\n",
                     baseName(pngPath).c_str());
        std::fclose(mtl);
    }

    FILE* obj = std::fopen(objPath.c_str(), "w");
    if (!obj) return false;

    std::string buffer;
    buffer.reserve(mesh.vertexCount() * 96 + mesh.indices.size() * 12);
    char line[192];

    std::snprintf(line, sizeof(line), "# BulletCam offline photogrammetry export\n");
    buffer += line;
    if (hasTexture) {
        std::snprintf(line, sizeof(line), "mtllib %s\n", baseName(mtlPath).c_str());
        buffer += line;
    }

    for (const Vec3& p : mesh.positions) {
        std::snprintf(line, sizeof(line), "v %.6f %.6f %.6f\n", p.x, p.y, p.z);
        buffer += line;
    }
    const bool hasNormals = mesh.normals.size() == mesh.positions.size();
    if (hasNormals) {
        for (const Vec3& n : mesh.normals) {
            std::snprintf(line, sizeof(line), "vn %.5f %.5f %.5f\n", n.x, n.y, n.z);
            buffer += line;
        }
    }
    if (hasTexture) {
        // OBJ's texture origin is bottom-left, ours is top-left.
        for (const Vec2& uv : mesh.uvs) {
            std::snprintf(line, sizeof(line), "vt %.6f %.6f\n", uv.x, 1.f - uv.y);
            buffer += line;
        }
        std::snprintf(line, sizeof(line), "usemtl bulletcam\n");
        buffer += line;
    }

    for (size_t t = 0; t < mesh.triangleCount(); ++t) {
        const uint32_t a = mesh.indices[t * 3 + 0] + 1;
        const uint32_t b = mesh.indices[t * 3 + 1] + 1;
        const uint32_t c = mesh.indices[t * 3 + 2] + 1;
        if (hasTexture && hasNormals)
            std::snprintf(line, sizeof(line), "f %u/%u/%u %u/%u/%u %u/%u/%u\n", a, a, a, b, b, b, c, c, c);
        else if (hasNormals)
            std::snprintf(line, sizeof(line), "f %u//%u %u//%u %u//%u\n", a, a, b, b, c, c);
        else
            std::snprintf(line, sizeof(line), "f %u %u %u\n", a, b, c);
        buffer += line;
    }

    const bool ok = std::fwrite(buffer.data(), 1, buffer.size(), obj) == buffer.size();
    std::fclose(obj);
    return ok;
}

bool writeGlb(const std::string& glbPath, const Mesh& mesh, const Texture& texture) {
    if (mesh.empty()) return false;
    const uint32_t vertexCount = static_cast<uint32_t>(mesh.positions.size());
    const uint32_t indexCount = static_cast<uint32_t>(mesh.indices.size());
    const bool hasNormals = mesh.normals.size() == mesh.positions.size();
    const bool hasUvs = mesh.uvs.size() == mesh.positions.size();

    std::vector<uint8_t> png;
    const bool hasTexture = hasUvs && !texture.empty() && encodePng(texture, png);

    // --- Binary chunk -------------------------------------------------------
    std::vector<uint8_t> bin;
    bin.reserve(static_cast<size_t>(vertexCount) * 32 + indexCount * 4 + png.size() + 64);

    struct BufferViewInfo {
        uint32_t offset = 0;
        uint32_t length = 0;
    };
    auto appendBlock = [&bin](const void* data, size_t length) {
        BufferViewInfo info;
        info.offset = static_cast<uint32_t>(bin.size());
        info.length = static_cast<uint32_t>(length);
        const uint8_t* bytes = static_cast<const uint8_t*>(data);
        bin.insert(bin.end(), bytes, bytes + length);
        padTo4(bin, 0);
        return info;
    };

    const BufferViewInfo positionView =
        appendBlock(mesh.positions.data(), sizeof(Vec3) * vertexCount);
    BufferViewInfo normalView;
    if (hasNormals) normalView = appendBlock(mesh.normals.data(), sizeof(Vec3) * vertexCount);
    BufferViewInfo uvView;
    if (hasUvs) uvView = appendBlock(mesh.uvs.data(), sizeof(Vec2) * vertexCount);
    const BufferViewInfo indexView = appendBlock(mesh.indices.data(), sizeof(uint32_t) * indexCount);
    BufferViewInfo imageView;
    if (hasTexture) imageView = appendBlock(png.data(), png.size());

    BBox bounds = meshBounds(mesh);

    // --- JSON chunk ---------------------------------------------------------
    std::ostringstream json;
    json.precision(9);
    json << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"BulletCam offline photogrammetry\"},";
    json << "\"scene\":0,\"scenes\":[{\"nodes\":[0]}],";
    json << "\"nodes\":[{\"mesh\":0,\"name\":\"capture\"}],";

    json << "\"buffers\":[{\"byteLength\":" << bin.size() << "}],";

    json << "\"bufferViews\":[";
    json << "{\"buffer\":0,\"byteOffset\":" << positionView.offset
         << ",\"byteLength\":" << positionView.length << ",\"target\":34962}";
    int viewIndex = 1;
    int normalViewIndex = -1, uvViewIndex = -1, imageViewIndex = -1;
    if (hasNormals) {
        json << ",{\"buffer\":0,\"byteOffset\":" << normalView.offset
             << ",\"byteLength\":" << normalView.length << ",\"target\":34962}";
        normalViewIndex = viewIndex++;
    }
    if (hasUvs) {
        json << ",{\"buffer\":0,\"byteOffset\":" << uvView.offset
             << ",\"byteLength\":" << uvView.length << ",\"target\":34962}";
        uvViewIndex = viewIndex++;
    }
    json << ",{\"buffer\":0,\"byteOffset\":" << indexView.offset
         << ",\"byteLength\":" << indexView.length << ",\"target\":34963}";
    const int indexViewIndex = viewIndex++;
    if (hasTexture) {
        json << ",{\"buffer\":0,\"byteOffset\":" << imageView.offset
             << ",\"byteLength\":" << imageView.length << "}";
        imageViewIndex = viewIndex++;
    }
    json << "],";

    json << "\"accessors\":[";
    json << "{\"bufferView\":0,\"componentType\":5126,\"count\":" << vertexCount
         << ",\"type\":\"VEC3\",\"min\":[" << bounds.lo.x << "," << bounds.lo.y << ","
         << bounds.lo.z << "],\"max\":[" << bounds.hi.x << "," << bounds.hi.y << ","
         << bounds.hi.z << "]}";
    int accessorIndex = 1;
    int normalAccessor = -1, uvAccessor = -1;
    if (hasNormals) {
        json << ",{\"bufferView\":" << normalViewIndex << ",\"componentType\":5126,\"count\":"
             << vertexCount << ",\"type\":\"VEC3\"}";
        normalAccessor = accessorIndex++;
    }
    if (hasUvs) {
        json << ",{\"bufferView\":" << uvViewIndex << ",\"componentType\":5126,\"count\":"
             << vertexCount << ",\"type\":\"VEC2\"}";
        uvAccessor = accessorIndex++;
    }
    json << ",{\"bufferView\":" << indexViewIndex << ",\"componentType\":5125,\"count\":"
         << indexCount << ",\"type\":\"SCALAR\"}";
    const int indexAccessor = accessorIndex++;
    json << "],";

    if (hasTexture) {
        json << "\"images\":[{\"bufferView\":" << imageViewIndex << ",\"mimeType\":\"image/png\"}],";
        json << "\"samplers\":[{\"magFilter\":9729,\"minFilter\":9987,\"wrapS\":33071,\"wrapT\":33071}],";
        json << "\"textures\":[{\"sampler\":0,\"source\":0}],";
    }

    json << "\"materials\":[{\"name\":\"bulletcam\",\"doubleSided\":true,\"pbrMetallicRoughness\":{";
    if (hasTexture) json << "\"baseColorTexture\":{\"index\":0},";
    json << "\"metallicFactor\":0.0,\"roughnessFactor\":0.95}}],";

    json << "\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0";
    if (normalAccessor >= 0) json << ",\"NORMAL\":" << normalAccessor;
    if (uvAccessor >= 0) json << ",\"TEXCOORD_0\":" << uvAccessor;
    json << "},\"indices\":" << indexAccessor << ",\"material\":0,\"mode\":4}]}]";
    json << "}";

    std::string jsonText = json.str();
    std::vector<uint8_t> jsonChunk(jsonText.begin(), jsonText.end());
    padTo4(jsonChunk, ' ');

    // --- Container ----------------------------------------------------------
    std::vector<uint8_t> glb;
    glb.reserve(jsonChunk.size() + bin.size() + 32);
    const char magic[4] = {'g', 'l', 'T', 'F'};
    glb.insert(glb.end(), magic, magic + 4);
    appendLittleEndian32(glb, 2);
    appendLittleEndian32(glb, static_cast<uint32_t>(12 + 8 + jsonChunk.size() + 8 + bin.size()));
    appendLittleEndian32(glb, static_cast<uint32_t>(jsonChunk.size()));
    const char jsonType[4] = {'J', 'S', 'O', 'N'};
    glb.insert(glb.end(), jsonType, jsonType + 4);
    glb.insert(glb.end(), jsonChunk.begin(), jsonChunk.end());
    appendLittleEndian32(glb, static_cast<uint32_t>(bin.size()));
    const char binType[4] = {'B', 'I', 'N', '\0'};
    glb.insert(glb.end(), binType, binType + 4);
    glb.insert(glb.end(), bin.begin(), bin.end());

    FILE* f = std::fopen(glbPath.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(glb.data(), 1, glb.size(), f) == glb.size();
    std::fclose(f);
    return ok;
}

}  // namespace bc
