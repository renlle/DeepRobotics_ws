import sys
import os
from dotenv import load_dotenv

# 在导入界面及启动子进程前加载工作区配置；不覆盖显式导出的环境变量。
load_dotenv(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.env")))

# 强制选用 FastDDS 作为通信中间件，以利用 C++ 节点之间高带宽的高速共享内存 (SHM) 通信
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

# 强制使用独立的 ROS_DOMAIN_ID 进行通信隔离，防止与机器狗底层内部节点或局域网内其他设备产生 /tf 冲突
domain_id = "42"
try:
    import yaml
    current_dir = os.path.dirname(os.path.abspath(__file__))
    yaml_path = os.path.abspath(os.path.join(current_dir, "../../config/device_parameters.yaml"))
    if os.path.exists(yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            config_data = yaml.safe_load(f)
            if config_data and "/**" in config_data:
                params = config_data["/**"].get("ros__parameters", {})
                if "ros_domain_id" in params:
                    domain_id = str(params["ros_domain_id"])
except Exception as e:
    print(f"Failed to read ros_domain_id from config, using default 42: {e}")

os.environ["ROS_DOMAIN_ID"] = domain_id

# 为 PyQt UI 进程指定专属的 FastDDS 配置，强制其仅使用 UDPv4 传输，从而避开 ARM64 架构下 Python C-bindings 订阅 C++ 共享内存的 Bug
current_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.abspath(os.path.join(current_dir, "../../config/fastdds_udp_only.xml"))
if os.path.exists(config_path):
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = config_path


from PyQt5.QtWidgets import QApplication

# 导入界面与后台解耦模块
# pyrefly: ignore [missing-import]
from main_window import RobotMainWindow
# pyrefly: ignore [missing-import]
from ros2_node import Ros2BackendThread

def main():
    # 1. 启动 Qt 应用
    from PyQt5.QtCore import Qt
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    
    # 2. 实例化界面 (View)
    window = RobotMainWindow()
    
    # 3. 实例化并启动后台 ROS2 通信线程 (Model)
    ros_thread = Ros2BackendThread()
    
    # 4. 连接后台 ROS 2 线程与前端 GUI 实例 of 跨线程信号槽
    ros_thread.signals.pose_updated.connect(window.update_hud)
    ros_thread.signals.log_msg.connect(window.append_log)
    
    # 连接定位状态更新信号，确保地图撤销和显示逻辑同步
    ros_thread.signals.status_updated.connect(window.handle_localization_status)
    ros_thread.signals.odom_distance_updated.connect(window.update_odom_distance)
    ros_thread.signals.ui_clear.connect(window.ui_clear_callback)
    
    # 重置槽连接：当前端触发重置信号时，通知后台清空历史 TF 缓存
    window.sig_reset_tf.connect(ros_thread.reset_tf)
    # 停止导航槽连接：通知后台发送空 /global_path 消息清除 RViz 全局路径残影
    window.sig_clear_nav.connect(ros_thread.clear_nav_viz)
    
    # 启动后台监听线程
    ros_thread.start()
    
    # 显示主界面
    window.show()
    
    # 5. 挂起主循环，程序结束时安全清理
    exit_code = app.exec_()
    
    # 关闭时挂起并回收后台线程
    ros_thread.stop()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
