#!/usr/bin/env python3
"""只清理当前工作区和 ROS 域的旧总控，不向其他节点发送信号。"""

import os
from pathlib import Path
import signal
import sys
import time


def find_processes(workspace, domain, executable):
    found = {}
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            args = proc.joinpath("cmdline").read_bytes().decode().rstrip("\0").split("\0")
            # 可执行入口可能直接运行，也可能由 Python 解释器运行。
            node = str(executable) in args[:2]
            launcher = any(
                Path(arg).name == "ros2"
                and args[index + 1:index + 4] == ["launch", "master_control", "navigation_command.launch.py"]
                for index, arg in enumerate(args[:2])
            )
            if not node and not launcher:
                continue
            env = dict(item.split("=", 1) for item in
                       proc.joinpath("environ").read_bytes().decode().split("\0") if "=" in item)
            if env.get("ROS_DOMAIN_ID", "0") != domain:
                continue
            if launcher and proc.joinpath("cwd").resolve() != workspace:
                continue
            # starttime 防止等待期间 PID 被其他进程复用。
            start = proc.joinpath("stat").read_text().rsplit(")", 1)[1].split()[19]
            found[int(proc.name)] = start
        except (FileNotFoundError, ProcessLookupError):
            continue
    return found


def stop_old(workspace, domain, executable):
    old = find_processes(workspace, domain, executable)
    if not old:
        print("未发现旧总控进程。", flush=True)
        return
    for sig, timeout in ((signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
        current = find_processes(workspace, domain, executable)
        remaining = {pid: start for pid, start in old.items() if current.get(pid) == start}
        if not remaining:
            return
        for pid in remaining:
            try:
                os.kill(pid, sig)
                print(f"清理旧总控 PID={pid}，信号={sig.name}", flush=True)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            current = find_processes(workspace, domain, executable)
            if not any(current.get(pid) == start for pid, start in old.items()):
                return
            time.sleep(0.1)
    raise RuntimeError("旧总控未退出，取消启动新总控")


if __name__ == "__main__":
    stop_old(Path(sys.argv[1]).resolve(), sys.argv[2], Path(sys.argv[3]))
