#pragma once
#include "core/types.h"

namespace bc {

// Pinhole intrinsics, following the OpenMVG / COLMAP pinhole model. ARCore's
// getImageIntrinsics() already reports this parameterisation for the CPU image,
// so no distortion terms are required.
struct Pinhole {
    float fx = 1.f, fy = 1.f, cx = 0.f, cy = 0.f;
    int width = 0, height = 0;

    Pinhole scaled(int newWidth, int newHeight) const {
        Pinhole k;
        const float sx = static_cast<float>(newWidth) / static_cast<float>(width);
        const float sy = static_cast<float>(newHeight) / static_cast<float>(height);
        k.fx = fx * sx;
        k.fy = fy * sy;
        // The 0.5 shift keeps the principal point aligned with texel centres.
        k.cx = (cx + 0.5f) * sx - 0.5f;
        k.cy = (cy + 0.5f) * sy - 0.5f;
        k.width = newWidth;
        k.height = newHeight;
        return k;
    }

    Mat3 matrix() const {
        Mat3 m = Mat3::zero();
        m(0, 0) = fx; m(0, 2) = cx;
        m(1, 1) = fy; m(1, 2) = cy;
        m(2, 2) = 1.f;
        return m;
    }
    Mat3 inverseMatrix() const {
        Mat3 m = Mat3::zero();
        m(0, 0) = 1.f / fx; m(0, 2) = -cx / fx;
        m(1, 1) = 1.f / fy; m(1, 2) = -cy / fy;
        m(2, 2) = 1.f;
        return m;
    }

    // Camera-space point -> pixel. Assumes p.z > 0.
    Vec2 project(const Vec3& p) const { return {fx * p.x / p.z + cx, fy * p.y / p.z + cy}; }
    // Pixel + depth -> camera-space point.
    Vec3 unproject(float px, float py, float depth) const {
        return {(px - cx) * depth / fx, (py - cy) * depth / fy, depth};
    }
    // Unit-Z ray through a pixel.
    Vec3 ray(float px, float py) const { return {(px - cx) / fx, (py - cy) / fy, 1.f}; }
};

// A registered view: an image plus the world->camera rigid transform recovered
// from ARCore's visual-inertial odometry.
struct View {
    Pinhole intrinsics;
    Mat3 R;      // world -> camera rotation
    Vec3 t;      // world -> camera translation
    Vec3 center; // camera centre in world space, == -R^T * t

    void setFromCameraToWorld(const Mat3& Rcw, const Vec3& C) {
        R = Rcw.transpose();
        center = C;
        t = -(R * C);
    }
    Vec3 worldToCamera(const Vec3& p) const { return R * p + t; }
    Vec3 cameraToWorld(const Vec3& p) const { return R.transpose() * (p - t); }
    // Unit-length world-space viewing direction through a pixel.
    Vec3 worldRay(float px, float py) const {
        return (R.transpose() * intrinsics.ray(px, py)).normalized();
    }
    Vec3 forward() const { return {R(2, 0), R(2, 1), R(2, 2)}; }

    View scaled(int width, int height) const {
        View v = *this;
        v.intrinsics = intrinsics.scaled(width, height);
        return v;
    }
};

// Relative pose taking a point from `ref` camera space into `src` camera space:
//   x_src = R_rel * x_ref + t_rel
inline void relativePose(const View& ref, const View& src, Mat3& Rrel, Vec3& trel) {
    Rrel = src.R * ref.R.transpose();
    trel = src.t - Rrel * ref.t;
}

// Fundamental matrix mapping points in `ref` to epipolar lines in `src`.
inline Mat3 fundamentalMatrix(const View& ref, const View& src) {
    Mat3 Rrel;
    Vec3 trel;
    relativePose(ref, src, Rrel, trel);
    const Mat3 E = Mat3::skew(trel) * Rrel;
    return src.intrinsics.inverseMatrix().transpose() * E * ref.intrinsics.inverseMatrix();
}

// Perpendicular distance from a pixel in `src` to the epipolar line induced by
// a pixel in `ref`. Used to prune descriptor matching to a 1-D search.
inline float epipolarDistance(const Mat3& F, const Vec2& refPx, const Vec2& srcPx) {
    const Vec3 line = F * Vec3{refPx.x, refPx.y, 1.f};
    const float n = std::sqrt(line.x * line.x + line.y * line.y);
    if (n < 1e-12f) return std::numeric_limits<float>::max();
    return std::fabs(line.x * srcPx.x + line.y * srcPx.y + line.z) / n;
}

}  // namespace bc
