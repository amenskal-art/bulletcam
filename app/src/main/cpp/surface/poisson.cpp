#include "surface/poisson.h"

#include <algorithm>

#include "core/log.h"
#include "core/parallel.h"
#include "surface/marching_tets.h"

namespace bc {
namespace {

struct Level {
    int nx = 0, ny = 0, nz = 0;
    float h = 1.f;
    std::vector<float> x, b, r;

    size_t index(int i, int j, int k) const { return (static_cast<size_t>(k) * ny + j) * nx + i; }
    size_t count() const { return static_cast<size_t>(nx) * ny * nz; }
};

// Red-black Gauss-Seidel. Boundary vertices stay at zero (Dirichlet), which is
// the right condition here because the indicator field is constant far from the
// samples.
void smooth(Level& level, int sweeps) {
    const float h2 = level.h * level.h;
    for (int s = 0; s < sweeps; ++s) {
        for (int colour = 0; colour < 2; ++colour) {
            parallelBlocks(level.nz - 2, [&](int kBegin, int kEnd) {
                for (int kk = kBegin; kk < kEnd; ++kk) {
                    const int k = kk + 1;
                    for (int j = 1; j + 1 < level.ny; ++j) {
                        const int start = 1 + (((k + j) + colour) & 1);
                        for (int i = start; i + 1 < level.nx; i += 2) {
                            const size_t c = level.index(i, j, k);
                            const float sum = level.x[c - 1] + level.x[c + 1] +
                                              level.x[c - level.nx] + level.x[c + level.nx] +
                                              level.x[c - static_cast<size_t>(level.nx) * level.ny] +
                                              level.x[c + static_cast<size_t>(level.nx) * level.ny];
                            level.x[c] = (sum - h2 * level.b[c]) / 6.f;
                        }
                    }
                }
            });
        }
    }
}

void computeResidual(Level& level) {
    const float invH2 = 1.f / (level.h * level.h);
    level.r.assign(level.count(), 0.f);
    parallelBlocks(level.nz - 2, [&](int kBegin, int kEnd) {
        for (int kk = kBegin; kk < kEnd; ++kk) {
            const int k = kk + 1;
            for (int j = 1; j + 1 < level.ny; ++j) {
                for (int i = 1; i + 1 < level.nx; ++i) {
                    const size_t c = level.index(i, j, k);
                    const float lap = (level.x[c - 1] + level.x[c + 1] + level.x[c - level.nx] +
                                       level.x[c + level.nx] +
                                       level.x[c - static_cast<size_t>(level.nx) * level.ny] +
                                       level.x[c + static_cast<size_t>(level.nx) * level.ny] -
                                       6.f * level.x[c]) *
                                      invH2;
                    level.r[c] = level.b[c] - lap;
                }
            }
        }
    });
}

void restrictResidual(const Level& fine, Level& coarse) {
    std::fill(coarse.b.begin(), coarse.b.end(), 0.f);
    parallelBlocks(coarse.nz, [&](int kBegin, int kEnd) {
        for (int k = kBegin; k < kEnd; ++k) {
            for (int j = 0; j < coarse.ny; ++j) {
                for (int i = 0; i < coarse.nx; ++i) {
                    if (i == 0 || j == 0 || k == 0 || i + 1 == coarse.nx || j + 1 == coarse.ny ||
                        k + 1 == coarse.nz)
                        continue;
                    // Full weighting over the 3x3x3 stencil.
                    float sum = 0.f, weightSum = 0.f;
                    for (int dk = -1; dk <= 1; ++dk) {
                        for (int dj = -1; dj <= 1; ++dj) {
                            for (int di = -1; di <= 1; ++di) {
                                const int fi = 2 * i + di, fj = 2 * j + dj, fk = 2 * k + dk;
                                if (fi < 0 || fj < 0 || fk < 0 || fi >= fine.nx || fj >= fine.ny ||
                                    fk >= fine.nz)
                                    continue;
                                const float w = static_cast<float>((2 - std::abs(di)) *
                                                                   (2 - std::abs(dj)) *
                                                                   (2 - std::abs(dk)));
                                sum += w * fine.r[fine.index(fi, fj, fk)];
                                weightSum += w;
                            }
                        }
                    }
                    coarse.b[coarse.index(i, j, k)] = weightSum > 0.f ? sum / weightSum : 0.f;
                }
            }
        }
    });
}

void prolongAndCorrect(const Level& coarse, Level& fine) {
    parallelBlocks(fine.nz, [&](int kBegin, int kEnd) {
        for (int k = kBegin; k < kEnd; ++k) {
            const float cz = static_cast<float>(k) * 0.5f;
            const int k0 = std::min(coarse.nz - 2, std::max(0, static_cast<int>(cz)));
            const float fz = clampf(cz - static_cast<float>(k0), 0.f, 1.f);
            for (int j = 0; j < fine.ny; ++j) {
                const float cy = static_cast<float>(j) * 0.5f;
                const int j0 = std::min(coarse.ny - 2, std::max(0, static_cast<int>(cy)));
                const float fy = clampf(cy - static_cast<float>(j0), 0.f, 1.f);
                for (int i = 0; i < fine.nx; ++i) {
                    if (i == 0 || j == 0 || k == 0 || i + 1 == fine.nx || j + 1 == fine.ny ||
                        k + 1 == fine.nz)
                        continue;
                    const float cx = static_cast<float>(i) * 0.5f;
                    const int i0 = std::min(coarse.nx - 2, std::max(0, static_cast<int>(cx)));
                    const float fx = clampf(cx - static_cast<float>(i0), 0.f, 1.f);
                    float acc = 0.f;
                    for (int dk = 0; dk < 2; ++dk) {
                        const float wz = dk ? fz : 1.f - fz;
                        for (int dj = 0; dj < 2; ++dj) {
                            const float wy = dj ? fy : 1.f - fy;
                            for (int di = 0; di < 2; ++di) {
                                const float wx = di ? fx : 1.f - fx;
                                acc += wx * wy * wz *
                                       coarse.x[coarse.index(i0 + di, j0 + dj, k0 + dk)];
                            }
                        }
                    }
                    fine.x[fine.index(i, j, k)] += acc;
                }
            }
        }
    });
}

void vCycle(std::vector<Level>& levels, size_t depth, int preSweeps, int postSweeps) {
    Level& level = levels[depth];
    if (depth + 1 == levels.size()) {
        smooth(level, 48);  // coarsest grid: iterate until it is effectively solved
        return;
    }
    smooth(level, preSweeps);
    computeResidual(level);
    Level& coarse = levels[depth + 1];
    restrictResidual(level, coarse);
    std::fill(coarse.x.begin(), coarse.x.end(), 0.f);
    vCycle(levels, depth + 1, preSweeps, postSweeps);
    prolongAndCorrect(coarse, level);
    smooth(level, postSweeps);
}

// Separable [1, 2, 1] blur; approximates the smoothing filter Kazhdan applies to
// the sample normals before taking the divergence.
void blurComponent(std::vector<float>& field, int nx, int ny, int nz) {
    std::vector<float> tmp(field.size());
    auto index = [&](int i, int j, int k) { return (static_cast<size_t>(k) * ny + j) * nx + i; };

    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const float a = field[index(std::max(0, i - 1), j, k)];
                const float b = field[index(i, j, k)];
                const float c = field[index(std::min(nx - 1, i + 1), j, k)];
                tmp[index(i, j, k)] = 0.25f * a + 0.5f * b + 0.25f * c;
            }
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const float a = tmp[index(i, std::max(0, j - 1), k)];
                const float b = tmp[index(i, j, k)];
                const float c = tmp[index(i, std::min(ny - 1, j + 1), k)];
                field[index(i, j, k)] = 0.25f * a + 0.5f * b + 0.25f * c;
            }
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const float a = field[index(i, j, std::max(0, k - 1))];
                const float b = field[index(i, j, k)];
                const float c = field[index(i, j, std::min(nz - 1, k + 1))];
                tmp[index(i, j, k)] = 0.25f * a + 0.5f * b + 0.25f * c;
            }
    field.swap(tmp);
}

}  // namespace

Mesh poissonReconstruct(const PointCloud& cloud, const ReconstructionOptions& options,
                        Progress& progress) {
    Mesh mesh;
    if (cloud.size() < 64) return mesh;

    BBox box = computeBounds(cloud);
    box.expand(0.08f);
    const float extent = std::max(1e-5f, box.maxExtent());
    const int resolution = std::max(48, options.poissonResolution);
    const float h = extent / static_cast<float>(resolution);
    const int pad = 4;

    ScalarGrid chi;
    chi.h = h;
    chi.origin = box.lo - Vec3{h, h, h} * static_cast<float>(pad);
    const Vec3 size = box.extent();
    chi.resize(std::max(16, static_cast<int>(std::ceil(size.x / h)) + 2 * pad + 1),
               std::max(16, static_cast<int>(std::ceil(size.y / h)) + 2 * pad + 1),
               std::max(16, static_cast<int>(std::ceil(size.z / h)) + 2 * pad + 1));
    BC_LOGI("Poisson grid %dx%dx%d, h=%.5f", chi.nx, chi.ny, chi.nz, h);
    progress.report(0.05f);

    const size_t cellCount = chi.values.size();
    std::vector<float> vx(cellCount, 0.f), vy(cellCount, 0.f), vz(cellCount, 0.f);
    std::vector<uint8_t> mask(cellCount, 0);

    // --- Splat oriented samples into the vector field ------------------------
    for (size_t p = 0; p < cloud.size(); ++p) {
        const Vec3 g = (cloud.positions[p] - chi.origin) / h;
        const int i0 = static_cast<int>(std::floor(g.x));
        const int j0 = static_cast<int>(std::floor(g.y));
        const int k0 = static_cast<int>(std::floor(g.z));
        if (i0 < 1 || j0 < 1 || k0 < 1 || i0 + 2 >= chi.nx || j0 + 2 >= chi.ny || k0 + 2 >= chi.nz)
            continue;
        const float fx = g.x - static_cast<float>(i0);
        const float fy = g.y - static_cast<float>(j0);
        const float fz = g.z - static_cast<float>(k0);
        const Vec3& n = cloud.normals[p];
        for (int dk = 0; dk < 2; ++dk) {
            const float wz = dk ? fz : 1.f - fz;
            for (int dj = 0; dj < 2; ++dj) {
                const float wy = dj ? fy : 1.f - fy;
                for (int di = 0; di < 2; ++di) {
                    const float w = (di ? fx : 1.f - fx) * wy * wz;
                    const size_t c = chi.index(i0 + di, j0 + dj, k0 + dk);
                    vx[c] += n.x * w;
                    vy[c] += n.y * w;
                    vz[c] += n.z * w;
                }
            }
        }
        // Flag the neighbourhood so extraction stays near real evidence.
        for (int dk = -2; dk <= 3; ++dk)
            for (int dj = -2; dj <= 3; ++dj)
                for (int di = -2; di <= 3; ++di) {
                    const int i = i0 + di, j = j0 + dj, k = k0 + dk;
                    if (!chi.inside(i, j, k)) continue;
                    mask[chi.index(i, j, k)] = 1;
                }
    }
    progress.report(0.2f);
    if (progress.cancelled()) return mesh;

    for (int pass = 0; pass < std::max(1, options.poissonSmoothIterations); ++pass) {
        blurComponent(vx, chi.nx, chi.ny, chi.nz);
        blurComponent(vy, chi.nx, chi.ny, chi.nz);
        blurComponent(vz, chi.nx, chi.ny, chi.nz);
    }
    progress.report(0.3f);

    // --- Build the multigrid hierarchy and solve laplacian(chi) = div(V) -----
    std::vector<Level> levels;
    {
        Level fine;
        fine.nx = chi.nx;
        fine.ny = chi.ny;
        fine.nz = chi.nz;
        fine.h = h;
        fine.x.assign(cellCount, 0.f);
        fine.b.assign(cellCount, 0.f);

        const float invTwoH = 1.f / (2.f * h);
        parallelBlocks(chi.nz - 2, [&](int kBegin, int kEnd) {
            for (int kk = kBegin; kk < kEnd; ++kk) {
                const int k = kk + 1;
                for (int j = 1; j + 1 < chi.ny; ++j) {
                    for (int i = 1; i + 1 < chi.nx; ++i) {
                        const size_t c = chi.index(i, j, k);
                        const size_t sy = static_cast<size_t>(chi.nx);
                        const size_t sz = static_cast<size_t>(chi.nx) * chi.ny;
                        fine.b[c] = ((vx[c + 1] - vx[c - 1]) + (vy[c + sy] - vy[c - sy]) +
                                     (vz[c + sz] - vz[c - sz])) *
                                    invTwoH;
                    }
                }
            }
        });
        levels.push_back(std::move(fine));
    }
    vx.clear(); vx.shrink_to_fit();
    vy.clear(); vy.shrink_to_fit();
    vz.clear(); vz.shrink_to_fit();

    while (levels.size() < 7) {
        const Level& fine = levels.back();
        if (std::min(fine.nx, std::min(fine.ny, fine.nz)) <= 12) break;
        Level coarse;
        coarse.nx = (fine.nx + 1) / 2;
        coarse.ny = (fine.ny + 1) / 2;
        coarse.nz = (fine.nz + 1) / 2;
        coarse.h = fine.h * 2.f;
        coarse.x.assign(coarse.count(), 0.f);
        coarse.b.assign(coarse.count(), 0.f);
        levels.push_back(std::move(coarse));
    }

    const int cycles = std::max(1, options.poissonVCycles);
    for (int c = 0; c < cycles; ++c) {
        if (progress.cancelled()) return mesh;
        vCycle(levels, 0, 2, 2);
        progress.report(0.3f + 0.5f * static_cast<float>(c + 1) / static_cast<float>(cycles));
    }
    chi.values = std::move(levels[0].x);
    levels.clear();

    // --- Iso value: the average the solution takes on the input samples ------
    double isoSum = 0.0;
    for (const Vec3& p : cloud.positions) isoSum += chi.sample(p);
    const float iso = static_cast<float>(isoSum / static_cast<double>(cloud.size()));
    progress.report(0.85f);

    mesh = extractIsosurface(chi, iso, mask);
    BC_LOGI("Poisson mesh: %zu vertices, %zu triangles", mesh.vertexCount(), mesh.triangleCount());
    progress.report(1.f);
    return mesh;
}

}  // namespace bc
