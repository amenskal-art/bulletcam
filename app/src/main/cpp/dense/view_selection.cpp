#include "dense/view_selection.h"

#include <algorithm>

namespace bc {
namespace {

// Bell curve centred on the angle that gives the best depth/occlusion trade-off.
float angleWeight(float angleDeg) {
    constexpr float kPreferred = 13.f;
    constexpr float kSigma = 9.f;
    if (angleDeg < 2.f || angleDeg > 55.f) return 0.f;
    const float d = (angleDeg - kPreferred) / kSigma;
    return std::exp(-0.5f * d * d);
}

}  // namespace

std::vector<std::vector<int>> selectSourceViews(const FrameStore& store, const SparseMap& sparse,
                                                int sourcesPerView) {
    const auto& frames = store.frames();
    const int n = static_cast<int>(frames.size());
    std::vector<std::vector<int>> result(static_cast<size_t>(n));
    if (n < 2) return result;

    const Vec3 target = sparse.bounds.valid() ? sparse.centroid : Vec3{0.f, 0.f, 0.f};

    for (int i = 0; i < n; ++i) {
        const View& vi = frames[static_cast<size_t>(i)].stereoView;
        const Vec3 dirI = (target - vi.center).normalized();
        std::vector<std::pair<float, int>> scored;
        scored.reserve(static_cast<size_t>(n - 1));

        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            const View& vj = frames[static_cast<size_t>(j)].stereoView;
            const Vec3 dirJ = (target - vj.center).normalized();
            const float angle = std::acos(clampf(dirI.dot(dirJ), -1.f, 1.f)) * 57.2957795f;
            const float w = angleWeight(angle);
            if (w <= 0.f) continue;

            // Views must broadly agree on where they are looking, otherwise the
            // plane-induced warp has almost no valid overlap.
            const float facing = vi.forward().dot(vj.forward());
            if (facing < 0.25f) continue;

            const float covisible = sparse.viewCount == n
                                        ? static_cast<float>(sparse.shared(i, j))
                                        : 0.f;
            // The +1 keeps geometry-only scoring usable on untextured objects
            // where the sparse stage finds almost nothing.
            scored.emplace_back(-w * (covisible + 1.f), j);
        }

        std::sort(scored.begin(), scored.end());
        const int take = std::min<int>(sourcesPerView, static_cast<int>(scored.size()));
        result[static_cast<size_t>(i)].reserve(static_cast<size_t>(take));
        for (int k = 0; k < take; ++k)
            result[static_cast<size_t>(i)].push_back(scored[static_cast<size_t>(k)].second);
    }
    return result;
}

}  // namespace bc
