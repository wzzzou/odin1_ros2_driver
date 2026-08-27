/*
本地扩展（非官方文件）：地图传输与信号退出的竞态防护。

问题（2026-08-27 真机实证）
------------------------------------------------------------------
官方在 process_command_file() 里用 detach 线程执行 lidar_save_map()
（SDK 默认生成超时 120 秒），而上游退出路径不检查传输状态，直接把
odinDevice 置空、调用 lidar_system_deinit() 并退出进程。

实测复现：建图 45 秒后触发 save_map=1，在 "Map save triggered" 后 4 毫秒
发送 SIGINT，结果是

  - 产生一个 0 字节的 .bin 地图文件，文件名格式与正常地图完全一致，
    肉眼无法区分，重定位时选中必然失败；
  - 日志中没有任何失败提示（detach 线程连 "Map transfer failed" 都来不及打印，
    进程已经退出）；
  - 设备侧状态机被打断，随后的启动卡在 "starting software connection"，
    USB 仍枚举但软连接不成功，必须给设备断电才能恢复。

对策
------------------------------------------------------------------
信号退出时若检测到传输仍在进行，先等待其结束（有上限），让 SDK 与设备把
本次传输走完，再继续原有的停流/反初始化流程。等待失败时明确告警，并清理
明显不完整的产物，避免把 0 字节文件留在地图目录里冒充正常地图。

等待上限取 kWaitSeconds，与 launch 中 host_sdk_node 的 sigterm_timeout
(20 秒) 对齐：超过它 launch 会自行升级到 SIGKILL，再等下去没有意义。

维护约定：本文件是本地新增。对官方文件的侵入以 ODIN_LOCAL_MAP_GUARD 标记。
*/
#ifndef ODIN_LOCAL_MAP_TRANSFER_GUARD_HPP
#define ODIN_LOCAL_MAP_TRANSFER_GUARD_HPP

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace odin_local {

class MapTransferGuard {
public:
    // detach 线程启动前调用，记录本次传输的目标文件。
    void begin(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        path_ = path;
    }

    // detach 线程结束时调用（无论成功与否）。
    void end(bool ok) {
        std::lock_guard<std::mutex> lock(mutex_);
        completed_ok_ = ok;
        path_.clear();
    }

    std::string path() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return path_;
    }

    // 轮询等待传输结束。is_running 由调用方提供（读官方的
    // g_map_transfer_in_progress）。返回 true 表示已结束，false 表示超时。
    //
    // 本函数只在主线程有序退出路径中调用：sleep、std::function 与 clock 均不再
    // 处于异步 signal handler 上下文。
    static bool wait_until_idle(const std::function<bool()>& is_running,
                                double timeout_s = kWaitSeconds) {
        using clock = std::chrono::steady_clock;
        const auto deadline = clock::now() + std::chrono::duration<double>(timeout_s);
        while (is_running()) {
            if (clock::now() >= deadline) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return true;
    }

    // 传输未走完时清理残留产物。只删明确不完整的文件（不存在或 0 字节），
    // 绝不动已经写出内容的地图，以免误删可用数据。
    // 返回是否删除了文件。
    bool cleanup_incomplete() const {
        std::string target;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            target = path_;
        }
        if (target.empty()) {
            return false;
        }
        std::error_code ec;
        if (!std::filesystem::exists(target, ec) || ec) {
            return false;
        }
        const auto size = std::filesystem::file_size(target, ec);
        if (ec || size != 0) {
            return false;
        }
        return std::filesystem::remove(target, ec) && !ec;
    }

    static constexpr double kWaitSeconds = 20.0;

private:
    mutable std::mutex mutex_;
    std::string path_;
    bool completed_ok_ = false;
};

inline MapTransferGuard& map_guard() {
    static MapTransferGuard instance;
    return instance;
}

}  // namespace odin_local

#endif  // ODIN_LOCAL_MAP_TRANSFER_GUARD_HPP
