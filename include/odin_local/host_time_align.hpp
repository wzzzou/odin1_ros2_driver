/*
本地扩展（非官方文件）：设备开机相对时基 -> 主机 Unix 时基的对齐。

背景（2026-08-27 真机实测，固件 0.13.1 + 驱动 0.14.0）
------------------------------------------------------------------
官方 g_use_host_ros_time 的三种模式，没有一种能同时满足"对齐主机时基"和
"保留传感器时序"：

  模式 0（原始开机相对戳）
      skew = 1787812355 s（约 56 年），jitter(sd) odom 0.029ms / imu 0.025ms
      时序完美，但与任何使用主机时钟的 ROS 节点都无法融合。

  模式 1（node->now()）
      skew = 0.000 s，jitter(sd) odom 10.918ms / imu 0.770ms
      对齐了，但时间戳在回调里取墙钟，把 USB 传输和调度延迟全部计入，
      odom 抖动放大 376 倍、imu 放大 31 倍。IMU 预积分与点云去畸变会受影响。

  模式 2（sensor - PTP offset）
      skew = 1646990288 s（约 52 年），没有对齐。
      根因不是单位错误而是语义错误：实测 ptp_sync_data_t.offset ≈ -1.408e8 秒，
      含义是"设备 Unix 时钟与主机 Unix 时钟之差"（设备时钟停在 2022-03），
      而数据流 header.stamp 是开机相对时间（实测约 824 秒）。两者不同源，
      相减没有物理意义。

本模式（3）的做法
------------------------------------------------------------------
      stamp = sensor_boot_ns + offset

offset 的估计用"窗口最小值 + 限速趋近"，而不是指数平均：

  instant = host_recv - sensor_boot = 真实偏移 + 传输延迟
  传输延迟恒为正，因此 instant 的**最小值**最接近真实偏移；取平均只会收敛到
  "真实偏移 + 平均延迟"。这是 PTP/NTP 的标准做法（minimum filtering）。

  用两段式窗口最小值维护，O(1) 空间：每 kWindowSeconds 轮换一次窗口，
  有效目标 = min(当前窗口最小, 上一窗口最小)，即最近 1~2 个窗口的下界。

  offset 以不超过 kMaxSlewPpm 的速率趋近目标，保证时间戳不会跳变或倒退。
  该速率远大于实测设备漂移（约 -36 ppm，每小时 -129 ms），因此能跟上漂移，
  又慢到不会把传输抖动引入逐帧间隔。

  首个实现曾用 30 秒时间常数的 EMA，实测 60 秒窗口内仍在收敛（表现为 +1585 ppm
  的假漂移），且稳态存在平均延迟偏置；故改为当前方案。

  - 保留传感器时序：offset 变化受限速约束，逐帧间隔仍由设备时钟决定。
  - 跟踪时钟漂移：窗口最小值持续更新，长时间运行不会累积误差。

维护约定：本文件是本地新增，官方升级时不会冲突。对官方文件的侵入仅限
include/host_sdk_sample.h 中 aligned_stamp_ns() 与 make_aligned_stamp() 各一个
分支，搜索 ODIN_LOCAL_TIME_ALIGN 可定位。
*/
#ifndef ODIN_LOCAL_HOST_TIME_ALIGN_HPP
#define ODIN_LOCAL_HOST_TIME_ALIGN_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <mutex>

namespace odin_local {

// g_use_host_ros_time 取该值时启用本模式。0/1/2 保留官方语义。
constexpr int kHostAlignedMode = 3;

class HostTimeAligner {
public:
    // 传入设备开机相对时间戳（纳秒），返回对齐到主机 Unix 时基的纳秒时间戳。
    // 多个流的回调可能并发调用，用互斥量保护状态；临界区只有几次浮点比较。
    uint64_t align(uint64_t sensor_boot_ns) {
        const double sensor_s = static_cast<double>(sensor_boot_ns) * 1e-9;
        const double host_s = host_now_seconds();
        const double instant = host_s - sensor_s;

        double offset;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!initialized_) {
                offset_ = instant;
                cur_min_ = instant;
                prev_min_ = instant;
                window_start_s_ = host_s;
                last_update_s_ = host_s;
                initialized_ = true;
            } else {
                cur_min_ = std::min(cur_min_, instant);
                if (host_s - window_start_s_ >= kWindowSeconds) {
                    prev_min_ = cur_min_;
                    cur_min_ = instant;
                    window_start_s_ = host_s;
                    if (windows_done_ < kFastLockWindows) {
                        ++windows_done_;
                    }
                }
                const double target = std::min(cur_min_, prev_min_);
                const double dt = host_s - last_update_s_;
                if (dt > 0.0) {
                    if (windows_done_ < kFastLockWindows) {
                        // 冷启动：首帧 instant 含缓冲积压，可能偏离真值上百毫秒。
                        // 限速趋近需要数百秒，期间时间戳持续偏移。因此前几个窗口
                        // 直接采用窗口最小值，快速完成粗对齐。
                        offset_ = target;
                    } else {
                        const double max_step = kMaxSlewPpm * 1e-6 * dt;
                        const double delta = target - offset_;
                        offset_ += std::clamp(delta, -max_step, max_step);
                    }
                    last_update_s_ = host_s;
                }
            }
            offset = offset_;
        }

        const double aligned_s = sensor_s + offset;
        if (aligned_s <= 0.0) {
            return 0ULL;
        }
        return static_cast<uint64_t>(aligned_s * 1e9);
    }

    double offset_seconds() {
        std::lock_guard<std::mutex> lock(mutex_);
        return offset_;
    }

private:
    static double host_now_seconds() {
        const auto now = std::chrono::system_clock::now().time_since_epoch();
        return std::chrono::duration<double>(now).count();
    }

    // 窗口轮换周期：有效最小值窗口为 5~10 秒。
    static constexpr double kWindowSeconds = 5.0;
    // 冷启动阶段直接采用窗口最小值的窗口数，之后转入限速跟踪。
    static constexpr int kFastLockWindows = 2;
    // 最大趋近速率 500 ppm = 0.5 ms/s；实测设备漂移约 36 ppm，余量充足。
    static constexpr double kMaxSlewPpm = 500.0;

    std::mutex mutex_;
    bool initialized_ = false;
    int windows_done_ = 0;
    double offset_ = 0.0;
    double cur_min_ = 0.0;
    double prev_min_ = 0.0;
    double window_start_s_ = 0.0;
    double last_update_s_ = 0.0;
};

// 全流共用一个对齐器：所有 topic 必须使用同一 offset，否则 topic 之间会产生
// 相对偏差，破坏融合。
inline HostTimeAligner& aligner() {
    static HostTimeAligner instance;
    return instance;
}

}  // namespace odin_local

#endif  // ODIN_LOCAL_HOST_TIME_ALIGN_HPP
