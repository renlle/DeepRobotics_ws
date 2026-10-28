#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace runtime_diag {
inline int64_t steady_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 热路径只记固定大小计数，不分配内存、不读取墙钟、不等待业务锁。
struct Counter {
    std::atomic<uint64_t> input{0}, output{0}, dropped{0}, work_ns{0}, max_ns{0};
    std::atomic<int64_t> last_input{0}, last_output{0}, queued{0}, source_lag_ns{-1};
    void receive() {
        input.fetch_add(1, std::memory_order_relaxed);
        last_input.store(steady_ns(), std::memory_order_relaxed);
    }
    void finish(int64_t started = 0) {
        const auto now = steady_ns();
        output.fetch_add(1, std::memory_order_relaxed);
        last_output.store(now, std::memory_order_relaxed);
        if (!started) return;
        const uint64_t elapsed = now - started;
        work_ns.fetch_add(elapsed, std::memory_order_relaxed);
        auto old = max_ns.load(std::memory_order_relaxed);
        while (old < elapsed && !max_ns.compare_exchange_weak(old, elapsed, std::memory_order_relaxed)) {}
    }
};

class Scope {
    Counter& counter_;
    int64_t start_;
    bool pending_;
public:
    explicit Scope(Counter& counter, bool pending = false)
        : counter_(counter), start_(steady_ns()), pending_(pending) {}
    ~Scope() {
        counter_.finish(start_);
        if (pending_) --counter_.queued;
    }
};

struct Snapshot {
    uint64_t input = 0, output = 0, dropped = 0, work = 0;
    std::string json(Counter& c, double seconds, int64_t now = steady_ns()) {
        const auto in = c.input.load(), out = c.output.load(), drop = c.dropped.load(), work_now = c.work_ns.load();
        const auto last_in = c.last_input.load(), last_out = c.last_output.load();
        std::ostringstream s;
        s << std::fixed << std::setprecision(2)
          << "{\"in_hz\":" << (in-input)/seconds << ",\"out_hz\":" << (out-output)/seconds
          << ",\"drop\":" << drop-dropped << ",\"pending\":" << c.queued.load()
          << ",\"work_ms\":" << (out > output ? (work_now-work)/1e6/(out-output) : 0.0)
          << ",\"max_ms\":" << c.max_ns.exchange(0)/1e6
          << ",\"input_silent_ms\":" << (last_in ? (now-last_in)/1e6 : -1.0)
          << ",\"output_silent_ms\":" << (last_out ? (now-last_out)/1e6 : -1.0)
          << ",\"source_lag_ms\":" << (c.source_lag_ns.load() < 0 ? -1.0 : c.source_lag_ns.load()/1e6) << "}";
        input=in; output=out; dropped=drop; work=work_now;
        return s.str();
    }
};
} // namespace runtime_diag
