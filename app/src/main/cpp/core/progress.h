#pragma once
#include <atomic>
#include <functional>
#include <string>

namespace bc {

// Reports pipeline progress back to the JVM. The callback is only ever invoked
// from the thread that entered JNI, never from a worker thread, so the JNIEnv*
// captured by the binding stays valid.
class Progress {
public:
    using Sink = std::function<void(const std::string&, float)>;

    Progress() = default;
    explicit Progress(Sink sink) : sink_(std::move(sink)) {}

    // Progress owns an atomic and therefore is neither copyable nor movable;
    // callers rebind the sink in place instead of assigning a fresh instance.
    void reset(Sink sink) {
        sink_ = std::move(sink);
        stage_ = "Idle";
        begin_ = 0.f;
        end_ = 1.f;
        cancelled_.store(false, std::memory_order_relaxed);
    }

    void setStage(const std::string& stage, float begin, float end) {
        stage_ = stage;
        begin_ = begin;
        end_ = end;
        report(0.f);
    }

    // `local` is the fraction complete within the current stage, in [0, 1].
    void report(float local) const {
        if (!sink_) return;
        const float f = begin_ + (end_ - begin_) * (local < 0.f ? 0.f : (local > 1.f ? 1.f : local));
        sink_(stage_, f);
    }

    void requestCancel() { cancelled_.store(true, std::memory_order_relaxed); }
    bool cancelled() const { return cancelled_.load(std::memory_order_relaxed); }

private:
    Sink sink_;
    std::string stage_ = "Idle";
    float begin_ = 0.f;
    float end_ = 1.f;
    std::atomic<bool> cancelled_{false};
};

}  // namespace bc
