#pragma once
#include <algorithm>
#include <atomic>
#include <functional>
#include <thread>
#include <vector>

namespace bc {

inline int hardwareThreads() {
    const unsigned n = std::thread::hardware_concurrency();
    return static_cast<int>(std::max(1u, std::min(n ? n : 4u, 8u)));
}

// Splits [0, count) into contiguous blocks, one per worker.
inline void parallelBlocks(int count, const std::function<void(int, int)>& body) {
    if (count <= 0) return;
    const int threads = std::min(hardwareThreads(), count);
    if (threads <= 1) {
        body(0, count);
        return;
    }
    std::vector<std::thread> pool;
    pool.reserve(static_cast<size_t>(threads));
    const int chunk = (count + threads - 1) / threads;
    for (int t = 0; t < threads; ++t) {
        const int begin = t * chunk;
        const int end = std::min(count, begin + chunk);
        if (begin >= end) break;
        pool.emplace_back([&body, begin, end] { body(begin, end); });
    }
    for (auto& th : pool) th.join();
}

// Dynamic work stealing over single indices; use when per-item cost varies a lot.
inline void parallelFor(int count, const std::function<void(int)>& body) {
    if (count <= 0) return;
    const int threads = std::min(hardwareThreads(), count);
    if (threads <= 1) {
        for (int i = 0; i < count; ++i) body(i);
        return;
    }
    std::atomic<int> cursor{0};
    std::vector<std::thread> pool;
    pool.reserve(static_cast<size_t>(threads));
    for (int t = 0; t < threads; ++t) {
        pool.emplace_back([&] {
            for (;;) {
                const int i = cursor.fetch_add(1, std::memory_order_relaxed);
                if (i >= count) return;
                body(i);
            }
        });
    }
    for (auto& th : pool) th.join();
}

}  // namespace bc
