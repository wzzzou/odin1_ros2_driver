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
统一解析到一个可写、与部署形态无关的运行时根目录。

优先复用官方已有的 odin_ros_driver::GetOdinRuntimeDir()（include/odin_calib_path.h），
它已经是 calib 主存储的路径来源，优先级为
    ODIN_CALIB_DIR -> ROS_HOME -> $HOME/.ros/odin_ros_driver -> /tmp/odin_ros_driver
launch 端 get_odin_runtime_dir() 也是同一套优先级。复用它而不是另写一份，
可以保证 C++、launch、calib、数据输出四者永远指向同一个位置。

额外提供 ODIN_DATA_DIR，优先级高于上面：录制数据体积很大（本仓历史 recorddata
已达 8.6 GB），需要单独放到大容量盘时用它覆盖，而不影响 calib 的位置。

不再写源码树，也不再写 share 目录。已存在于源码树的历史数据不迁移，只影响新数据。

维护约定：本文件是本地新增。对官方文件的侵入以 ODIN_LOCAL_RUNTIME_PATHS 标记。
*/
#ifndef ODIN_LOCAL_RUNTIME_PATHS_HPP
#define ODIN_LOCAL_RUNTIME_PATHS_HPP

#include <cstdlib>
#include <filesystem>
#include <string>

#include "odin_calib_path.h"

namespace odin_local {

// 运行时可写数据根目录。
inline std::filesystem::path runtime_root() {
    if (const char* env = std::getenv("ODIN_DATA_DIR")) {
        if (env[0] != '\0') {
            return std::filesystem::path(env);
        }
    }
    // 复用官方实现，保证与 calib 主存储、launch 端解析结果一致。
    return std::filesystem::path(odin_ros_driver::GetOdinRuntimeDir());
}

inline std::string runtime_subdir(const char* name) {
    return (runtime_root() / name).string();
}

}  // namespace odin_local

#endif  // ODIN_LOCAL_RUNTIME_PATHS_HPP
