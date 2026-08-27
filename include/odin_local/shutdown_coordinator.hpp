/*
本地扩展（非官方文件）：进程退出协调。

问题（2026-08-27 真机实证）
------------------------------------------------------------------
上游驱动把完整清理放在 SIGINT/SIGTERM handler 中执行，其中包含 ROS 日志、
文件系统访问、sleep、线程 join、SDK 停流/deinit 和 rclcpp::shutdown。这些操作
都不应在异步信号上下文中运行；SIGINT 与 SIGTERM 还可能落到不同 SDK/ROS
线程并交叉进入清理。设备回调另有多处 exit(1) 与 pkill -f，会跳过设备清理并
误杀本机其他 launch（包括 SentryNav 的 RViz）。

设计
------------------------------------------------------------------
在 main() 创建任何 ROS/SDK 线程前屏蔽 SIGINT/SIGTERM，再启动一个 sigwait
线程同步接收信号。sigwait 线程只提交 ShutdownRequest；设备回调遇到致命错误
也只提交请求并返回。所有地图等待、线程 join、停流、deinit、ROS shutdown
统一由 main 线程执行。

首个信号请求有序退出；宽限期内的重复信号忽略；超过宽限期后再次发信号才
强制 _exit(1)。宽限期设为 35 秒，覆盖地图传输等待上限 20 秒与正常 deinit，
并与 launch 的 40 秒 SIGTERM 超时配套。

请求状态压在一个 uint32_t 原子中，保证 reason/exit code/signal number 作为
同一快照发布；第一个请求获胜，避免回调错误与用户信号互相覆盖。

维护约定：本文件是本地新增。对官方文件的侵入以
ODIN_LOCAL_SHUTDOWN_COORDINATOR 标记。
*/
#ifndef ODIN_LOCAL_SHUTDOWN_COORDINATOR_HPP
#define ODIN_LOCAL_SHUTDOWN_COORDINATOR_HPP

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <pthread.h>
#include <signal.h>
#include <thread>
#include <unistd.h>

namespace odin_local {

enum class ShutdownReason : std::uint8_t {
    kNone = 0,
    kSignal = 1,
    kUsbTransport = 2,
    kFirmwareRead = 3,
    kFirmwareTooOld = 4,
    kUnexpectedDeviceState = 5,
    kConnectionTimeout = 6,
    kDeviceSetupFailed = 7,
    kSdkInitFailed = 8,
    kUnhandledException = 9,
    kRosShutdown = 10,
};

struct ShutdownRequest {
    ShutdownReason reason{ShutdownReason::kNone};
    int exit_code{0};
    int signal_number{0};
};

class ShutdownCoordinator {
public:
    bool block_shutdown_signals() {
        bool expected = false;
        if (!signals_blocked_.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel)) {
            return true;
        }

        const sigset_t signals = shutdown_signal_set();
        const int mask_rc = pthread_sigmask(SIG_BLOCK, &signals, nullptr);
        if (mask_rc != 0) {
            signals_blocked_.store(false, std::memory_order_release);
            return false;
        }
        return true;
    }

    bool start_signal_waiter() {
        if (!block_shutdown_signals()) {
            return false;
        }

        bool expected = false;
        if (!signal_waiter_started_.compare_exchange_strong(
                expected, true, std::memory_order_acq_rel)) {
            return true;
        }

        const sigset_t signals = shutdown_signal_set();
        try {
            std::thread([this, signals]() mutable {
                for (;;) {
                    int signum = 0;
                    if (sigwait(&signals, &signum) == 0) {
                        on_signal(signum);
                    }
                }
            }).detach();
        } catch (...) {
            signal_waiter_started_.store(false, std::memory_order_release);
            return false;
        }
        return true;
    }

    bool request(ShutdownReason reason, int exit_code, int signal_number = 0) {
        if (reason == ShutdownReason::kNone) {
            return false;
        }
        std::uint32_t expected = 0;
        const std::uint32_t desired = pack(reason, exit_code, signal_number);
        return state_.compare_exchange_strong(
            expected, desired, std::memory_order_acq_rel, std::memory_order_acquire);
    }

    bool requested() const {
        return state_.load(std::memory_order_acquire) != 0;
    }

    ShutdownRequest snapshot() const {
        return unpack(state_.load(std::memory_order_acquire));
    }

    static const char* reason_name(ShutdownReason reason) {
        switch (reason) {
            case ShutdownReason::kSignal: return "signal";
            case ShutdownReason::kUsbTransport: return "USB transport is below USB 3.x";
            case ShutdownReason::kFirmwareRead: return "firmware version read failed";
            case ShutdownReason::kFirmwareTooOld: return "firmware version is too old";
            case ShutdownReason::kUnexpectedDeviceState: return "unexpected streaming device state";
            case ShutdownReason::kConnectionTimeout: return "device connection timed out";
            case ShutdownReason::kDeviceSetupFailed: return "device setup failed";
            case ShutdownReason::kSdkInitFailed: return "lidar SDK initialization failed";
            case ShutdownReason::kUnhandledException: return "unhandled initialization exception";
            case ShutdownReason::kRosShutdown: return "ROS context stopped";
            case ShutdownReason::kNone: return "none";
        }
        return "unknown";
    }

    static constexpr double kForceExitGraceSeconds = 35.0;

private:
    static sigset_t shutdown_signal_set() {
        sigset_t signals;
        sigemptyset(&signals);
        sigaddset(&signals, SIGINT);
        sigaddset(&signals, SIGTERM);
        return signals;
    }

    void on_signal(int signum) {
        const std::int64_t now_ns = monotonic_nanoseconds();
        std::int64_t expected = 0;
        if (first_signal_ns_.compare_exchange_strong(
                expected, now_ns, std::memory_order_acq_rel)) {
            (void)request(ShutdownReason::kSignal, 0, signum);
            return;
        }

        const double elapsed = static_cast<double>(now_ns - expected) / 1e9;
        if (elapsed >= kForceExitGraceSeconds) {
            // sigwait 在普通线程上下文运行；这里只作为清理确实卡死后的最终兜底。
            static constexpr char message[] =
                "Odin shutdown exceeded grace period; forcing process exit\n";
            const ssize_t written =
                ::write(STDERR_FILENO, message, sizeof(message) - 1);
            (void)written;
            _exit(1);
        }
        // 宽限期内重复 Ctrl-C/SIGTERM：让 main 线程继续有序清理。
    }

    static std::int64_t monotonic_nanoseconds() {
        struct timespec ts {};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
    }

    static std::uint32_t pack(ShutdownReason reason, int exit_code, int signal_number) {
        return static_cast<std::uint32_t>(reason)
             | ((static_cast<std::uint32_t>(exit_code) & 0xffU) << 8U)
             | ((static_cast<std::uint32_t>(signal_number) & 0xffU) << 16U);
    }

    static ShutdownRequest unpack(std::uint32_t state) {
        ShutdownRequest request;
        request.reason = static_cast<ShutdownReason>(state & 0xffU);
        request.exit_code = static_cast<int>((state >> 8U) & 0xffU);
        request.signal_number = static_cast<int>((state >> 16U) & 0xffU);
        return request;
    }

    std::atomic<std::uint32_t> state_{0};
    std::atomic<std::int64_t> first_signal_ns_{0};
    std::atomic<bool> signals_blocked_{false};
    std::atomic<bool> signal_waiter_started_{false};
};

inline ShutdownCoordinator& shutdown_coordinator() {
    static ShutdownCoordinator instance;
    return instance;
}

}  // namespace odin_local

#endif  // ODIN_LOCAL_SHUTDOWN_COORDINATOR_HPP
