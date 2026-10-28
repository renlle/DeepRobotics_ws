# 控制台设备参数目录修复

## 问题

从项目根目录运行：

```bash
python3 -u robot_console/robot_console/app.py
```

点击“选择设备参数配置文件”时，控制台错误地查找：

```text
/home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/src/DeepRobotics_ws/config/device_profiles
```

当前项目源码直接位于工作空间根目录，不存在 `src/DeepRobotics_ws` 这一层，因此弹出“参数配置模板目录不存在”。

## 修改位置

修改文件：

```text
robot_console/robot_console/main_window.py
```

### 1. 启动时的默认参数路径

在 `MainWindow` 初始化代码中，将默认模板和活动参数文件从旧的 `src/DeepRobotics_ws/config` 路径改为工作空间根目录下的 `config`：

```python
self.default_profile_path = os.path.join(
    self.ws_path, 'config', 'device_profiles', 'stationary.yaml'
)
self.target_parameters_path = os.path.join(
    self.ws_path, 'config', 'device_parameters.yaml'
)
```

对应实际路径：

```text
/home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/config/device_profiles/stationary.yaml
/home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/config/device_parameters.yaml
```

### 2. “选择设备参数配置文件”按钮目录

在 `select_profile()` 方法中，将文件选择器目录修改为：

```python
profiles_dir = os.path.abspath(
    os.path.join(self.ws_path, 'config', 'device_profiles')
)
```

对应实际目录：

```text
/home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/config/device_profiles
```

该目录当前包含：

```text
mobile.yaml
stationary.yaml
```

## 生效方式

如果控制台已经运行，需要先退出旧进程，再重新启动：

```bash
cd /home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws
source /opt/ros/humble/setup.bash
python3 -u robot_console/robot_console/app.py
```

重新点击“选择设备参数配置文件”后，文件选择器将直接打开：

```text
/home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws/config/device_profiles
```
