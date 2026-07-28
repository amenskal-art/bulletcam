#pragma once
#include <vector>

#include "core/types.h"

namespace bc {

// Uniform scalar grid. Vertex (i, j, k) sits at origin + h * (i, j, k).
struct ScalarGrid {
    int nx = 0, ny = 0, nz = 0;
    float h = 1.f;
    Vec3 origin;
    std::vector<float> values;

    void resize(int x, int y, int z) {
        nx = x;
        ny = y;
        nz = z;
        values.assign(static_cast<size_t>(x) * y * z, 0.f);
    }
    size_t index(int x, int y, int z) const {
        return (static_cast<size_t>(z) * ny + y) * nx + x;
    }
    float& at(int x, int y, int z) { return values[index(x, y, z)]; }
    float at(int x, int y, int z) const { return values[index(x, y, z)]; }
    bool inside(int x, int y, int z) const {
        return x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz;
    }
    Vec3 position(int x, int y, int z) const {
        return origin + Vec3{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)} * h;
    }

    float sample(const Vec3& p) const {
        const Vec3 g = (p - origin) / h;
        const int x0 = std::min(nx - 2, std::max(0, static_cast<int>(std::floor(g.x))));
        const int y0 = std::min(ny - 2, std::max(0, static_cast<int>(std::floor(g.y))));
        const int z0 = std::min(nz - 2, std::max(0, static_cast<int>(std::floor(g.z))));
        const float fx = clampf(g.x - static_cast<float>(x0), 0.f, 1.f);
        const float fy = clampf(g.y - static_cast<float>(y0), 0.f, 1.f);
        const float fz = clampf(g.z - static_cast<float>(z0), 0.f, 1.f);
        float acc = 0.f;
        for (int dz = 0; dz < 2; ++dz) {
            const float wz = dz ? fz : 1.f - fz;
            for (int dy = 0; dy < 2; ++dy) {
                const float wy = dy ? fy : 1.f - fy;
                for (int dx = 0; dx < 2; ++dx) {
                    const float wx = dx ? fx : 1.f - fx;
                    acc += wx * wy * wz * at(x0 + dx, y0 + dy, z0 + dz);
                }
            }
        }
        return acc;
    }

    Vec3 gradient(int x, int y, int z) const {
        const int xm = std::max(0, x - 1), xp = std::min(nx - 1, x + 1);
        const int ym = std::max(0, y - 1), yp = std::min(ny - 1, y + 1);
        const int zm = std::max(0, z - 1), zp = std::min(nz - 1, z + 1);
        return {(at(xp, y, z) - at(xm, y, z)) / (static_cast<float>(xp - xm) * h),
                (at(x, yp, z) - at(x, ym, z)) / (static_cast<float>(yp - ym) * h),
                (at(x, y, zp) - at(x, y, zm)) / (static_cast<float>(zp - zm) * h)};
    }
};

}  // namespace bc
