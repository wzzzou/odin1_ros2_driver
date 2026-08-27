/*
本地扩展（非官方文件）：运行时数据目录的解析。

问题（2026-08-27 实证）
------------------------------------------------------------------
官方在 main() 里这样推导 recorddata / log / map 的输出位置：

    char* ros_workspace = std::getenv("COLCON_PREFIX_PATH");
    size_t pos = workspace_path.find("/install");
    if (pos != npos) data_dir = workspace_path.substr(0, pos) + "/src/odin_ros_driver/recorddata";
    else             data_dir = <ament share>/odin_ros_driver + "/recorddata";

三个问题：

  1. 正常路径下运行时数据被写进**源码树**。地图、日志、录制数据混进 git 工作区，
     只能靠 .gitignore 兜住；源码目录只读或不存在时（NUC 部署只装 install/）直接失败。

  2. 靠 "/install" 做字符串截断非常脆弱。本仓 src/odin_ros_driver/ 下存在一个历史
     遗留的包内 install/，一旦 source 到它，COLCON_PREFIX_PATH 会指向
     ".../src/odin_ros_driver/install"，截断后得到 ".../src/odin_ros_driver"，
     再拼上 "/src/odin_ros_driver/recorddata" 就变成双层嵌套的
     ".../src/odin_ros_driver/src/odin_ros_driver/recorddata"。
     实测这条路径在约 11 分钟内写入 7.1 GB，几乎撑爆磁盘。

  3. fallback 分支写 ament share 目录。安装态目录通常应视为只读，系统级安装
     （/opt/ros/...）下会直接失败。

对策
------------------------------------------------------------------
统一解析到一个可写、与部署形态无关的运行时根目录，优先级：

    1. 环境变量 ODIN_DATA_DIR（显式指定，便于 NUC / 多机 / 录制盘分离）
    2. $HOME/.ros/odin_ros_driver  （默认；与官方 calib 主存储位置一致）
    3. 当前工作目录下的 odin_ros_driver_data（HOME 不可用时的兜底）

不再写源码树，也不再写 share 目录。已存在于源码树的历史数据不迁移，只影响新数据。

维护约定：本文件是本地新增。对官方文件的侵入以 ODIN_LOCAL_RUNTIME_PATHS 标记。
*/
#ifndef ODIN_LOCAL_RUNTIME_PATHS_HPP
#define ODIN_LOCAL_RUNTIME_PATHS_HPP

#include <cstdlib>
#include <filesystem>
#include <string>

namespace odin_local {

// 运行时可写数据根目录。
inline std::filesystem::path runtime_root() {
    if (const char* env = std::getenv("ODIN_DATA_DIR")) {
        if (env[0] != '\0') {
            return std::filesystem::path(env);
        }
    }
    if (const char* home = std::getenv("HOME")) {
        if (home[0] != '\0') {
            return std::filesystem::path(home) / ".ros" / "odin_ros_driver";
        }
    }
    return std::filesystem::current_path() / "odin_ros_driver_data";
}

inline std::string runtime_subdir(const char* name) {
    return (runtime_root() / name).string();
}

}  // namespace odin_local

#endif  // ODIN_LOCAL_RUNTIME_PATHS_HPP
