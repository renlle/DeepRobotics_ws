#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_interface.h>
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

// 内存定时器不初始化 ROS、不连接 DDS，也不发布任何机器人话题。
class Timers : public tf2_ros::CreateTimerInterface {
public:
  tf2_ros::TimerHandle createTimer(rclcpp::Clock::SharedPtr,
    const tf2::Duration &, tf2_ros::TimerCallbackType callback) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    callbacks[++next] = std::move(callback);
    return next;
  }
  void cancel(const tf2_ros::TimerHandle & handle) override {remove(handle);}
  void reset(const tf2_ros::TimerHandle &) override {}
  void remove(const tf2_ros::TimerHandle & handle) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    callbacks.erase(handle);
  }
  void fire()
  {
    std::map<tf2_ros::TimerHandle, tf2_ros::TimerCallbackType> copy;
    {std::lock_guard<std::mutex> lock(mutex); copy = callbacks;}
    for (const auto & entry : copy) {entry.second(entry.first);}
  }
private:
  std::mutex mutex;
  tf2_ros::TimerHandle next = 0;
  std::map<tf2_ros::TimerHandle, tf2_ros::TimerCallbackType> callbacks;
};

int main()
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  auto timers = std::make_shared<Timers>();
  buffer.setCreateTimerInterface(timers);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base";
  tf.transform.rotation.w = 1.0;
  tf.header.stamp.sec = 1;
  buffer.setTransform(tf, "test");
  std::atomic<int> current{1}, completed{0};
  std::atomic<bool> running{true};
  std::thread writer([&]() {
    for (int i = 2; i <= 20000; ++i) {
      tf.header.stamp.sec = i;
      tf.transform.translation.x = i;
      buffer.setTransform(tf, "test");
      current = i;
      std::this_thread::yield();
    }
    running = false;
  });
  std::thread requester([&]() {
    // 通过虚接口调用，覆盖 Nav2 MessageFilter 使用的虚函数跳板。
    tf2_ros::AsyncBufferInterface & async = buffer;
    for (int i = 0; i < 20000; ++i) {
      auto future = async.waitForTransform("odom", "base",
        tf2::TimePoint(std::chrono::seconds(current + 1)), std::chrono::seconds(1),
        [&](const tf2_ros::TransformStampedFuture & value) {
          try {value.get(); ++completed;} catch (const tf2::TransformException &) {}
        });
      if (i % 3 == 0) {async.cancel(future);}
      std::this_thread::yield();
    }
  });
  std::thread timeout([&]() {
    while (running) {timers->fire(); std::this_thread::yield();}
  });
  writer.join(); requester.join(); timeout.join();
  timers->fire();
  auto latest = buffer.lookupTransform("odom", "base", tf2::TimePointZero);
  assert(latest.transform.translation.x == 20000);
  assert(completed > 0);
  std::cout << "PASS: 20000 TF updates and requests, cancel/timeout, latest pose="
            << latest.transform.translation.x << ", completed=" << completed << std::endl;
}
