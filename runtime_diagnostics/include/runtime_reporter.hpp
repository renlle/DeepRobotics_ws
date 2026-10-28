#pragma once
#include "runtime_metrics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <vector>

namespace runtime_diag {
// 每模块每 5 秒一条小消息；诊断使用 best-effort，不能反压控制链路。
class Reporter {
    std::string module_;
    std::vector<std::string> names_;
    std::vector<std::unique_ptr<Counter>> counters_;
    std::vector<Snapshot> previous_;
    int64_t last_ = steady_ns();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
public:
    Reporter(rclcpp::Node* node, std::string module, std::initializer_list<const char*> names,
             std::function<void()> sample = {})
        : module_(std::move(module)) {
        for (auto name : names) {
            names_.emplace_back(name);
            counters_.push_back(std::make_unique<Counter>());
            previous_.emplace_back();
        }
        publisher_ = node->create_publisher<std_msgs::msg::String>(
            "/runtime/performance", rclcpp::QoS(1).best_effort());
        timer_ = node->create_wall_timer(std::chrono::seconds(5), [this, sample]() {
            if (sample) sample();
            auto now = steady_ns();
            double dt = (now-last_)/1e9;
            last_ = now;
            std::ostringstream s;
            s << "{\"module\":\"" << module_ << "\",\"window_s\":" << dt << ",\"streams\":{";
            for (size_t i=0; i<names_.size(); ++i) {
                if (i) s << ',';
                s << '"' << names_[i] << "\":" << previous_[i].json(*counters_[i], dt, now);
            }
            s << "}}";
            std_msgs::msg::String msg;
            msg.data = s.str();
            publisher_->publish(msg);
        });
    }
    Counter& at(size_t index) { return *counters_[index]; }
};
} // namespace runtime_diag
