#include <tf2_ros/buffer.h>
#include <dlfcn.h>
#include <cstdio>
#include <cstdlib>
#include <mutex>

namespace {
// Humble 0.25.23 的 TF 请求锁与定时器锁存在反向获取路径。
// 仅在 Nav2 进程外层串行化这些入口；具体变换、超时和取消仍调用系统实现。
std::recursive_mutex & guard()
{
  static std::recursive_mutex mutex;
  return mutex;
}

template<typename Function>
Function original(const char * symbol)
{
  auto address = dlsym(RTLD_NEXT, symbol);
  if (!address) {
    std::fprintf(stderr, "nav_tf2_guard: missing symbol %s: %s\n", symbol, dlerror());
    std::abort();
  }
  return reinterpret_cast<Function>(address);
}
}  // namespace

bool tf2::BufferCore::setTransform(
  const geometry_msgs::msg::TransformStamped & transform,
  const std::string & authority, bool is_static)
{
  using Function = bool (*)(BufferCore *, const geometry_msgs::msg::TransformStamped &,
    const std::string &, bool);
  static auto call = original<Function>(
    "_ZN3tf210BufferCore12setTransformERKN13geometry_msgs3msg17TransformStamped_ISaIvEEERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEb");
  std::lock_guard<std::recursive_mutex> lock(guard());
  return call(this, transform, authority, is_static);
}

tf2_ros::TransformStampedFuture tf2_ros::Buffer::waitForTransform(
  const std::string & target, const std::string & source,
  const tf2::TimePoint & time, const tf2::Duration & timeout,
  TransformReadyCallback callback)
{
  using Function = TransformStampedFuture (*)(Buffer *, const std::string &,
    const std::string &, const tf2::TimePoint &, const tf2::Duration &, TransformReadyCallback);
  static auto call = original<Function>(
    "_ZN7tf2_ros6Buffer16waitForTransformERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES8_RKNSt6chrono10time_pointINS9_3_V212system_clockENS9_8durationIlSt5ratioILl1ELl1000000000EEEEEERKSG_St8functionIFvRKNS_22TransformStampedFutureEEE");
  std::lock_guard<std::recursive_mutex> lock(guard());
  return call(this, target, source, time, timeout, std::move(callback));
}

void tf2_ros::Buffer::cancel(const TransformStampedFuture & future)
{
  using Function = void (*)(Buffer *, const TransformStampedFuture &);
  static auto call = original<Function>(
    "_ZN7tf2_ros6Buffer6cancelERKNS_22TransformStampedFutureE");
  std::lock_guard<std::recursive_mutex> lock(guard());
  call(this, future);
}

void tf2_ros::Buffer::timerCallback(
  const TimerHandle & handle,
  std::shared_ptr<std::promise<geometry_msgs::msg::TransformStamped>> promise,
  TransformStampedFuture future, TransformReadyCallback callback)
{
  using Function = void (*)(Buffer *, const TimerHandle &,
    std::shared_ptr<std::promise<geometry_msgs::msg::TransformStamped>>,
    TransformStampedFuture, TransformReadyCallback);
  static auto call = original<Function>(
    "_ZN7tf2_ros6Buffer13timerCallbackERKmSt10shared_ptrISt7promiseIN13geometry_msgs3msg17TransformStamped_ISaIvEEEEENS_22TransformStampedFutureESt8functionIFvRKSC_EE");
  std::lock_guard<std::recursive_mutex> lock(guard());
  call(this, handle, std::move(promise), std::move(future), std::move(callback));
}
