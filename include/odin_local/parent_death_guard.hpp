/*
本地扩展（非官方文件）：ROS 2 launch 子进程父死亡保护。

Humble 的 launch_service 收到只投递给 launch PID 的 SIGTERM 时，监督进程可能
在分发 Shutdown 事件前退出，留下 host_sdk_sample 与辅助节点成为孤儿。Odin
驱动会继续占用 USB，后续实例无法接管设备。

run_with_parent_death_signal() 用于一个极薄的 Node(prefix=...) 可执行程序：
  1. 记录 launch 父 PID；
  2. prctl(PR_SET_PDEATHSIG, SIGTERM)，由内核在父进程死亡时通知本子进程；
  3. 再核对父 PID，封闭“读取 PPID 与 prctl 之间父进程已死”的竞态；
  4. 忽略 SIGPIPE，避免 launch 输出管道断开时日志写入直接杀死清理进程；
  5. execvp 原节点，PID 不变，PDEATHSIG 设置保留。

保护仅作用于本次 launch 的直接子进程，不扫描进程名，不会误杀 SentryNav
或其他 launch 的 RViz。
*/
#ifndef ODIN_LOCAL_PARENT_DEATH_GUARD_HPP
#define ODIN_LOCAL_PARENT_DEATH_GUARD_HPP

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/types.h>
#include <unistd.h>

namespace odin_local {

inline int run_with_parent_death_signal(int argc, char* argv[]) {
    int program_index = 1;
    const bool block_shutdown_signals =
        argc >= 3 && std::strcmp(argv[1], "--block-shutdown-signals") == 0;
    if (block_shutdown_signals) {
        program_index = 2;
    }
    if (argc <= program_index) {
        std::fprintf(
            stderr,
            "usage: odin_parent_death_guard [--block-shutdown-signals] <program> [args...]\n");
        return 64;
    }

    // host_sdk_sample 会在 main() 中用 sigwait 同步接管这两个信号。提前屏蔽可
    // 封闭 guard exec 到 C++ main() 之间父进程死亡/用户 Ctrl-C 的极小窗口。
    // 普通辅助节点不使用此选项，仍保持其框架默认信号处理。
    if (block_shutdown_signals) {
        sigset_t shutdown_signals;
        sigemptyset(&shutdown_signals);
        sigaddset(&shutdown_signals, SIGINT);
        sigaddset(&shutdown_signals, SIGTERM);
        if (sigprocmask(SIG_BLOCK, &shutdown_signals, nullptr) != 0) {
            std::fprintf(stderr, "odin_parent_death_guard: signal mask failed: %s\n",
                         std::strerror(errno));
            return 69;
        }
    }

    const pid_t expected_parent = getppid();
    if (expected_parent <= 1) {
        std::fprintf(stderr, "odin_parent_death_guard: launch parent is already gone\n");
        return 70;
    }

    if (prctl(PR_SET_PDEATHSIG, SIGTERM) != 0) {
        std::fprintf(stderr, "odin_parent_death_guard: PR_SET_PDEATHSIG failed: %s\n",
                     std::strerror(errno));
        return 71;
    }

    if (getppid() != expected_parent) {
        std::fprintf(stderr, "odin_parent_death_guard: launch parent died during setup\n");
        return 72;
    }

    struct sigaction ignore_pipe {};
    ignore_pipe.sa_handler = SIG_IGN;
    sigemptyset(&ignore_pipe.sa_mask);
    if (sigaction(SIGPIPE, &ignore_pipe, nullptr) != 0) {
        std::fprintf(stderr, "odin_parent_death_guard: SIGPIPE setup failed: %s\n",
                     std::strerror(errno));
        return 73;
    }

    execvp(argv[program_index], &argv[program_index]);
    const int saved_errno = errno;
    std::fprintf(stderr, "odin_parent_death_guard: execvp(%s) failed: %s\n",
                 argv[program_index], std::strerror(saved_errno));
    return saved_errno == ENOENT ? 127 : 126;
}

}  // namespace odin_local

#endif  // ODIN_LOCAL_PARENT_DEATH_GUARD_HPP
