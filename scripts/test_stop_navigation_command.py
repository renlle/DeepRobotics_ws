"""用独立临时进程验证清理范围和超时回收，不创建 ROS 节点。"""

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import shutil
import re

spec = importlib.util.spec_from_file_location(
    "stop_navigation_command", Path(__file__).with_name("stop_navigation_command.py"))
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class StopNavigationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.exe = self.root / "navigation_command_node"
        self.exe.write_text(
            "import signal,sys,time\n"
            "if '--ignore-term' in sys.argv: signal.signal(signal.SIGTERM,signal.SIG_IGN)\n"
            "print('ready',flush=True)\n"
            "time.sleep(120)\n")
        self.children = []

    def spawn(self, domain="197", path=None, args=(), cwd=None):
        child = subprocess.Popen(
            [sys.executable, str(path or self.exe), *args],
            env=dict(os.environ, ROS_DOMAIN_ID=domain), cwd=cwd or self.root,
            stdout=subprocess.PIPE, text=True)
        self.children.append(child)
        self.assertEqual(child.stdout.readline().strip(), "ready")
        return child

    def tearDown(self):
        for child in self.children:
            if child.poll() is None:
                child.kill()
            child.wait()
            child.stdout.close()
        self.temp.cleanup()

    def test_only_current_workspace_and_domain(self):
        old = self.spawn()
        other_domain = self.spawn("198")
        other_exe = self.root / "other" / "navigation_command_node"
        other_exe.parent.mkdir()
        other_exe.write_text(self.exe.read_text())
        other_workspace = self.spawn(path=other_exe)
        launcher_exe = self.root / "ros2"
        launcher_exe.write_text(self.exe.read_text())
        launch_args = ("launch", "master_control", "navigation_command.launch.py")
        launcher = self.spawn(path=launcher_exe, args=launch_args)
        other_launcher = self.spawn(path=launcher_exe, args=launch_args, cwd=other_exe.parent)
        helper.stop_old(self.root, "197", self.exe)
        self.assertEqual(old.wait(timeout=2), -signal.SIGTERM)
        self.assertEqual(launcher.wait(timeout=2), -signal.SIGTERM)
        for child in (other_domain, other_workspace, other_launcher):
            self.assertIsNone(child.poll())
        # 第二次清理无匹配进程也应成功，且不影响其他域。
        helper.stop_old(self.root, "197", self.exe)

    def test_force_kill_after_grace_period(self):
        old = self.spawn(args=("--ignore-term",))
        start = time.monotonic()
        helper.stop_old(self.root, "197", self.exe)
        self.assertGreaterEqual(time.monotonic() - start, 5)
        self.assertEqual(old.wait(timeout=2), -signal.SIGKILL)

    def test_launcher_replaces_old_node_and_reaps_new_node(self):
        # 用临时工作区和睡眠进程执行真实shell流程，屏蔽GUI和ROS命令。
        repo = Path(__file__).resolve().parents[1]
        shutil.copyfile(repo / "start_app.sh", self.root / "start_app.sh")
        (self.root / "install").mkdir()
        (self.root / "install/setup.bash").write_text("true\n")
        (self.root / "scripts").mkdir()
        shutil.copyfile(repo / "scripts/stop_navigation_command.py",
                        self.root / "scripts/stop_navigation_command.py")
        prefix = self.root / "fake_master_control"
        self.exe = prefix / "lib/master_control/navigation_command_node"
        self.exe.parent.mkdir(parents=True)
        self.exe.write_text(f"#!{sys.executable}\nimport time\nprint('ready',flush=True)\ntime.sleep(120)\n")
        self.exe.chmod(0o755)
        old = self.spawn(domain="42")
        shell = '''
ros2() {
  [[ "$*" == "pkg prefix master_control" ]] || return 1
  echo "$FAKE_PREFIX"
}
python3() {
  if [[ "$1" == "-u" ]]; then
    echo "GUI_STUB_EXIT"
    return 0
  fi
  command python3 "$@"
}
export -f ros2 python3
bash start_app.sh
'''
        result = subprocess.run(["bash", "-c", shell], cwd=self.root,
                                env=dict(os.environ, FAKE_PREFIX=str(prefix)),
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(old.wait(timeout=2), -signal.SIGTERM)
        self.assertIn("GUI_STUB_EXIT", result.stdout)
        match = re.search(r"已重新启动总控导航节点（PID=(\d+)）", result.stdout)
        self.assertIsNotNone(match, result.stdout)
        self.assertFalse(Path("/proc", match.group(1)).exists())


if __name__ == "__main__":
    unittest.main()
