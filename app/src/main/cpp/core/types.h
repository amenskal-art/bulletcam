#pragma once
// Minimal, dependency-free linear algebra for the reconstruction core.
// Conventions follow the standard computer-vision (OpenCV / OpenMVG) frame:
//   * camera looks down +Z, +X right, +Y down
//   * world -> camera is  x_cam = R * x_world + t
//   * pixels are (col, row) with the origin at the centre of the top-left texel
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace bc {

struct Vec2 {
    float x = 0.f, y = 0.f;
    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    float norm() const { return std::sqrt(x * x + y * y); }
};

struct Vec3 {
    float x = 0.f, y = 0.f, z = 0.f;
    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    float operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float squaredNorm() const { return x * x + y * y + z * z; }
    float norm() const { return std::sqrt(squaredNorm()); }
    Vec3 normalized() const {
        const float n = norm();
        return n > 1e-20f ? Vec3{x / n, y / n, z / n} : Vec3{0.f, 0.f, 0.f};
    }
    bool isFinite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

inline Vec3 cwiseMin(const Vec3& a, const Vec3& b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
inline Vec3 cwiseMax(const Vec3& a, const Vec3& b) {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

// Row-major 3x3.
struct Mat3 {
    float m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float& operator()(int r, int c) { return m[r * 3 + c]; }
    float operator()(int r, int c) const { return m[r * 3 + c]; }

    static Mat3 identity() { return Mat3(); }
    static Mat3 zero() {
        Mat3 r;
        for (int i = 0; i < 9; ++i) r.m[i] = 0.f;
        return r;
    }
    static Mat3 diag(float a, float b, float c) {
        Mat3 r = zero();
        r.m[0] = a; r.m[4] = b; r.m[8] = c;
        return r;
    }
    // Skew-symmetric matrix such that skew(v) * w == v.cross(w).
    static Mat3 skew(const Vec3& v) {
        Mat3 r = zero();
        r(0, 1) = -v.z; r(0, 2) = v.y;
        r(1, 0) = v.z;  r(1, 2) = -v.x;
        r(2, 0) = -v.y; r(2, 1) = v.x;
        return r;
    }

    Vec3 operator*(const Vec3& v) const {
        return {m[0] * v.x + m[1] * v.y + m[2] * v.z,
                m[3] * v.x + m[4] * v.y + m[5] * v.z,
                m[6] * v.x + m[7] * v.y + m[8] * v.z};
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r = zero();
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k) {
                const float a = (*this)(i, k);
                if (a == 0.f) continue;
                for (int j = 0; j < 3; ++j) r(i, j) += a * o(k, j);
            }
        return r;
    }
    Mat3 operator*(float s) const {
        Mat3 r;
        for (int i = 0; i < 9; ++i) r.m[i] = m[i] * s;
        return r;
    }
    Mat3 operator-(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 9; ++i) r.m[i] = m[i] - o.m[i];
        return r;
    }
    Mat3 transpose() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r(i, j) = (*this)(j, i);
        return r;
    }
    float det() const {
        return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
               m[2] * (m[3] * m[7] - m[4] * m[6]);
    }
    Mat3 inverse() const {
        const float d = det();
        Mat3 r = zero();
        if (std::fabs(d) < 1e-20f) return r;
        const float invd = 1.f / d;
        r(0, 0) = (m[4] * m[8] - m[5] * m[7]) * invd;
        r(0, 1) = (m[2] * m[7] - m[1] * m[8]) * invd;
        r(0, 2) = (m[1] * m[5] - m[2] * m[4]) * invd;
        r(1, 0) = (m[5] * m[6] - m[3] * m[8]) * invd;
        r(1, 1) = (m[0] * m[8] - m[2] * m[6]) * invd;
        r(1, 2) = (m[2] * m[3] - m[0] * m[5]) * invd;
        r(2, 0) = (m[3] * m[7] - m[4] * m[6]) * invd;
        r(2, 1) = (m[1] * m[6] - m[0] * m[7]) * invd;
        r(2, 2) = (m[0] * m[4] - m[1] * m[3]) * invd;
        return r;
    }
    // Re-orthonormalises a rotation that has drifted (Gram-Schmidt).
    Mat3 orthonormalized() const {
        Vec3 r0{m[0], m[1], m[2]};
        Vec3 r1{m[3], m[4], m[5]};
        r0 = r0.normalized();
        r1 = (r1 - r0 * r0.dot(r1)).normalized();
        const Vec3 r2 = r0.cross(r1);
        Mat3 r;
        r(0, 0) = r0.x; r(0, 1) = r0.y; r(0, 2) = r0.z;
        r(1, 0) = r1.x; r(1, 1) = r1.y; r(1, 2) = r1.z;
        r(2, 0) = r2.x; r(2, 1) = r2.y; r(2, 2) = r2.z;
        return r;
    }
};

// Jacobi eigen-decomposition of a symmetric 3x3 matrix.
// Eigenvalues are returned ascending; eigenvectors are the rows of `vec`.
inline void symmetricEigen(const Mat3& in, Vec3& val, Mat3& vec) {
    float a[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) a[i][j] = in(i, j);
    float v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    for (int sweep = 0; sweep < 32; ++sweep) {
        float off = 0.f;
        for (int i = 0; i < 3; ++i)
            for (int j = i + 1; j < 3; ++j) off += a[i][j] * a[i][j];
        if (off < 1e-24f) break;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::fabs(a[p][q]) < 1e-24f) continue;
                const float theta = (a[q][q] - a[p][p]) / (2.f * a[p][q]);
                const float sign = theta >= 0.f ? 1.f : -1.f;
                const float t = sign / (std::fabs(theta) + std::sqrt(theta * theta + 1.f));
                const float c = 1.f / std::sqrt(t * t + 1.f);
                const float s = t * c;
                for (int k = 0; k < 3; ++k) {
                    const float akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const float apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const float vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }

    int order[3] = {0, 1, 2};
    const float d[3] = {a[0][0], a[1][1], a[2][2]};
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j)
            if (d[order[j]] < d[order[i]]) std::swap(order[i], order[j]);

    for (int i = 0; i < 3; ++i) {
        val[i] = d[order[i]];
        for (int k = 0; k < 3; ++k) vec(i, k) = v[k][order[i]];
    }
}

struct BBox {
    Vec3 lo{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
            std::numeric_limits<float>::max()};
    Vec3 hi{-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
            -std::numeric_limits<float>::max()};
    void extend(const Vec3& p) { lo = cwiseMin(lo, p); hi = cwiseMax(hi, p); }
    bool valid() const { return hi.x >= lo.x; }
    Vec3 center() const { return (lo + hi) * 0.5f; }
    Vec3 extent() const { return hi - lo; }
    float maxExtent() const {
        const Vec3 e = extent();
        return std::max(e.x, std::max(e.y, e.z));
    }
    void expand(float relative) {
        const Vec3 e = extent() * relative;
        lo = lo - e;
        hi = hi + e;
    }
};

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Deterministic xorshift RNG. PatchMatch needs a fast per-thread generator and
// determinism makes reconstruction results reproducible for a given capture set.
struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed = 0x9E3779B9u) : s(seed ? seed : 0x9E3779B9u) {}
    uint32_t next() {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return s;
    }
    float uniform() { return static_cast<float>(next() & 0xFFFFFF) / 16777216.f; }
    float uniform(float lo, float hi) { return lo + (hi - lo) * uniform(); }
};

}  // namespace bc
