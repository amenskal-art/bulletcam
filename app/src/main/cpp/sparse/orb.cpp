#include "sparse/orb.h"

#include <algorithm>
#include <mutex>

#include "core/parallel.h"

namespace bc {
namespace {

constexpr int kPatchRadius = 15;     // intensity-centroid + BRIEF patch radius
constexpr int kBorder = kPatchRadius + 4;
constexpr int kDescriptorBits = 256;

// Bresenham circle of radius 3 used by FAST-9.
constexpr int kCircleX[16] = {0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1};
constexpr int kCircleY[16] = {-3, -3, -2, -1, 0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3};

struct BriefPair {
    int8_t ax, ay, bx, by;
};

// The BRIEF sampling pattern is generated once from a fixed seed with an
// isotropic Gaussian spread over the patch, as in the original BRIEF paper.
// A fixed seed keeps descriptors identical across runs and devices.
const std::vector<BriefPair>& briefPattern() {
    static const std::vector<BriefPair> pattern = [] {
        std::vector<BriefPair> p;
        p.reserve(kDescriptorBits);
        Rng rng(0x5EED1234u);
        const float sigma = kPatchRadius / 2.4f;
        auto gaussian = [&rng, sigma]() {
            // Box-Muller, clamped to the patch.
            const float u1 = std::max(1e-6f, rng.uniform());
            const float u2 = rng.uniform();
            const float g = std::sqrt(-2.f * std::log(u1)) * std::cos(6.2831853f * u2) * sigma;
            return static_cast<int8_t>(std::lround(clampf(g, -kPatchRadius, kPatchRadius)));
        };
        while (static_cast<int>(p.size()) < kDescriptorBits) {
            BriefPair pair{gaussian(), gaussian(), gaussian(), gaussian()};
            if (pair.ax == pair.bx && pair.ay == pair.by) continue;
            p.push_back(pair);
        }
        return p;
    }();
    return pattern;
}

// Returns the FAST-9 score (0 if not a corner).
float fastScore(const ImageU8& img, int x, int y, int threshold) {
    const int center = img.at(x, y);
    const int bright = center + threshold;
    const int dark = center - threshold;

    // Cheap rejection on the four cardinal circle points: at least three of them
    // must agree in sign for nine contiguous pixels to be possible.
    int brighter = 0, darker = 0;
    for (int i = 0; i < 16; i += 4) {
        const int v = img.at(x + kCircleX[i], y + kCircleY[i]);
        if (v > bright) ++brighter;
        else if (v < dark) ++darker;
    }
    if (brighter < 3 && darker < 3) return 0.f;

    int values[16];
    for (int i = 0; i < 16; ++i) values[i] = img.at(x + kCircleX[i], y + kCircleY[i]);

    auto contiguous = [&](bool wantBright) {
        int run = 0, best = 0;
        for (int i = 0; i < 24; ++i) {
            const int v = values[i & 15];
            const bool hit = wantBright ? (v > bright) : (v < dark);
            run = hit ? run + 1 : 0;
            best = std::max(best, run);
            if (best >= 9) return true;
        }
        return false;
    };
    if (!contiguous(true) && !contiguous(false)) return 0.f;

    float score = 0.f;
    for (int i = 0; i < 16; ++i) score += std::fabs(static_cast<float>(values[i] - center));
    return score;
}

float centroidAngle(const ImageU8& img, int x, int y) {
    float m01 = 0.f, m10 = 0.f;
    for (int dy = -kPatchRadius; dy <= kPatchRadius; ++dy) {
        const int span = static_cast<int>(
            std::sqrt(static_cast<float>(kPatchRadius * kPatchRadius - dy * dy)));
        for (int dx = -span; dx <= span; ++dx) {
            const float v = static_cast<float>(img.at(x + dx, y + dy));
            m10 += dx * v;
            m01 += dy * v;
        }
    }
    return std::atan2(m01, m10);
}

Descriptor describe(const ImageU8& img, const Keypoint& kp) {
    const auto& pattern = briefPattern();
    const float c = std::cos(kp.angle);
    const float s = std::sin(kp.angle);
    const int x = static_cast<int>(std::lround(kp.x));
    const int y = static_cast<int>(std::lround(kp.y));

    Descriptor d{0, 0, 0, 0};
    for (int i = 0; i < kDescriptorBits; ++i) {
        const BriefPair& p = pattern[static_cast<size_t>(i)];
        const int ax = x + static_cast<int>(std::lround(c * p.ax - s * p.ay));
        const int ay = y + static_cast<int>(std::lround(s * p.ax + c * p.ay));
        const int bx = x + static_cast<int>(std::lround(c * p.bx - s * p.by));
        const int by = y + static_cast<int>(std::lround(s * p.bx + c * p.by));
        if (img.at(ax, ay) < img.at(bx, by)) d[static_cast<size_t>(i >> 6)] |= 1ull << (i & 63);
    }
    return d;
}

}  // namespace

FeatureSet detectAndDescribe(const ImageU8& gray, int maxFeatures, int fastThreshold) {
    FeatureSet out;
    if (gray.width < 2 * kBorder + 2 || gray.height < 2 * kBorder + 2) return out;

    // Grid buckets keep features spread out; a dense corner cluster on one edge
    // of the object would otherwise starve the rest of the silhouette.
    const int cellSize = 24;
    const int cols = std::max(1, gray.width / cellSize);
    const int rows = std::max(1, gray.height / cellSize);
    const int perCell = std::max(2, maxFeatures / (cols * rows) + 1);

    std::vector<std::vector<Keypoint>> cells(static_cast<size_t>(cols) * rows);
    std::mutex cellMutex;

    parallelBlocks(gray.height - 2 * kBorder, [&](int begin, int end) {
        std::vector<std::vector<Keypoint>> local(static_cast<size_t>(cols) * rows);
        for (int yy = begin; yy < end; ++yy) {
            const int y = yy + kBorder;
            for (int x = kBorder; x < gray.width - kBorder; ++x) {
                const float score = fastScore(gray, x, y, fastThreshold);
                if (score <= 0.f) continue;
                Keypoint kp;
                kp.x = static_cast<float>(x);
                kp.y = static_cast<float>(y);
                kp.score = score;
                const int cx = std::min(cols - 1, x / cellSize);
                const int cy = std::min(rows - 1, y / cellSize);
                local[static_cast<size_t>(cy) * cols + cx].push_back(kp);
            }
        }
        std::lock_guard<std::mutex> lock(cellMutex);
        for (size_t i = 0; i < local.size(); ++i)
            cells[i].insert(cells[i].end(), local[i].begin(), local[i].end());
    });

    std::vector<Keypoint> selected;
    selected.reserve(static_cast<size_t>(maxFeatures));
    for (auto& cell : cells) {
        if (cell.empty()) continue;
        const size_t keep = std::min(cell.size(), static_cast<size_t>(perCell));
        std::partial_sort(cell.begin(), cell.begin() + static_cast<long>(keep), cell.end(),
                          [](const Keypoint& a, const Keypoint& b) { return a.score > b.score; });
        // Suppress neighbours inside the cell so descriptors stay distinctive.
        for (size_t i = 0; i < keep; ++i) {
            bool tooClose = false;
            for (size_t j = 0; j < i; ++j) {
                const float dx = cell[i].x - cell[j].x;
                const float dy = cell[i].y - cell[j].y;
                if (dx * dx + dy * dy < 9.f) { tooClose = true; break; }
            }
            if (!tooClose) selected.push_back(cell[i]);
        }
    }

    if (static_cast<int>(selected.size()) > maxFeatures) {
        std::partial_sort(selected.begin(), selected.begin() + maxFeatures, selected.end(),
                          [](const Keypoint& a, const Keypoint& b) { return a.score > b.score; });
        selected.resize(static_cast<size_t>(maxFeatures));
    }

    out.keypoints = std::move(selected);
    out.descriptors.resize(out.keypoints.size());
    parallelBlocks(static_cast<int>(out.keypoints.size()), [&](int begin, int end) {
        for (int i = begin; i < end; ++i) {
            Keypoint& kp = out.keypoints[static_cast<size_t>(i)];
            kp.angle = centroidAngle(gray, static_cast<int>(kp.x), static_cast<int>(kp.y));
            out.descriptors[static_cast<size_t>(i)] = describe(gray, kp);
        }
    });
    return out;
}

}  // namespace bc
