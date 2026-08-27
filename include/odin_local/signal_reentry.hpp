/*
本地扩展（非官方文件）：信号重入防护。

问题（2026-08-27 实证）
------------------------------------------------------------------
官方用 signal() 注册 SIGINT/SIGTERM。signal() 只在 handler 执行期间屏蔽
**同一个**信号，SIGINT 与 SIGTERM 可以交叉进入同一个 handler。

P0 第 8 轮实测：SIGTERM 的 handler 卡在 lidar_system_deinit() 内 28.6 秒，
期间 SIGINT 进入并并发执行同一段清理——日志出现两次 "Deinitializing lidar
system"，第二次因 odinDevice 已被前一个 handler 置空而跳过了 "Closing
device"。两个 handler 并发 join 线程、拆 publisher、调 SDK deinit 属未定义行为。

设计权衡
------------------------------------------------------------------
简单的"第二个信号立即 _exit"是错的：实测正常清理需 4~5.4 秒，用户快速连按
两次 Ctrl-C（间隔 0.3 秒）就会跳过全部清理，退化成 kill -9，可能重新引入
地图损坏与设备卡死。

因此按"距首个信号的时间"分流：

  首个信号                       -> kProceed，走完整清理
  距首个 < kGraceSeconds 的信号  -> kIgnore，忽略，让首个 handler 走完
  距首个 >= kGraceSeconds 的信号 -> kForceExit，认为清理确实卡住，强制退出

kGraceSeconds 取 10 秒：正常清理 5 秒内结束；地图传输中退出时实测传输仅需
0.7~1.1 秒（上限 20 秒是保守值），10 秒足以覆盖，同时不会让用户在真卡住时
等太久。launch 场景下 host_sdk_node 的 sigterm_timeout 为 20 秒，超时后
launch 自行升级到 SIGKILL，所以 grace 期设得比它更长没有意义。

async-signal-safety：本类只调用 clock_gettime(CLOCK_MONOTONIC)（POSIX 明确
列为 async-signal-safe）与原子读写，不分配内存、不取锁。

维护约定：本文件是本地新增。对官方文件的侵入以 ODIN_LOCAL_SIGNAL_REENTRY 标记。
*/
#ifndef ODIN_LOCAL_SIGNAL_REENTRY_HPP
#define ODIN_LOCAL_SIGNAL_REENTRY_HPP

#include <atomic>
#include <ctime>

namespace odin_local {

class SignalReentryGuard {
public:
    enum class Action {
        kProceed,    // 首个信号：执行完整清理
        kIgnore,     // 重入且仍在宽限期内：忽略
        kForceExit,  // 重入且已超宽限期：清理疑似卡死，强制退出
    };

    Action on_signal() {
        const double now = monotonic_seconds();
        // 内存序不能用 relaxed：内核可以把 SIGINT 与 SIGTERM 投递到**不同线程**，
        // 两个 handler 因而可能在不同 CPU 上执行。2026-08-27 实测，relaxed 下
        // 第二个 handler 读到 first_signal_s_ 的初值 0.0，(now - 0) 远大于宽限期，
        // 被误判成"清理卡死"而执行 _exit(1)，导致 SIGTERM 后 0.3 秒的一次 SIGINT
        // 就把完整清理打断（进程以 exit code 1 结束，未出现 "Closing device"）。
        // 这里用 release/acquire 配对，保证读到 entered_=true 时也能读到时间戳。
        if (entered_.exchange(true, std::memory_order_acq_rel)) {
            const double first = first_signal_s_.load(std::memory_order_acquire);
            // first == 0 表示首个 handler 还没来得及写入时间戳（exchange 与 store
            // 之间的极小窗口）。保守按忽略处理：宁可不强制退出，也不要打断
            // 正在进行的清理。
            if (first == 0.0 || now - first < kGraceSeconds) {
                return Action::kIgnore;
            }
            return Action::kForceExit;
        }
        first_signal_s_.store(now, std::memory_order_release);
        return Action::kProceed;
    }

    static constexpr double kGraceSeconds = 10.0;

private:
    // clock_gettime(CLOCK_MONOTONIC) 是 async-signal-safe，且不受系统时钟调整影响。
    static double monotonic_seconds() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<double>(ts.tv_sec) +
               static_cast<double>(ts.tv_nsec) * 1e-9;
    }

    std::atomic<bool> entered_{false};
    std::atomic<double> first_signal_s_{0.0};
};

inline SignalReentryGuard& reentry_guard() {
    static SignalReentryGuard instance;
    return instance;
}

}  // namespace odin_local

#endif  // ODIN_LOCAL_SIGNAL_REENTRY_HPP
