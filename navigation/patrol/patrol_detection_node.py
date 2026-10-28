#!/usr/bin/env python3
import os
import sys
import time
import shutil
import socket
import json
import struct
import threading
from datetime import datetime
import cv2
import numpy as np

import warnings
warnings.filterwarnings("ignore", category=UserWarning, message=".*CUDA initialization.*")
warnings.filterwarnings("ignore", category=UserWarning, message=".*CUDA unknown error.*")
warnings.filterwarnings("ignore", category=UserWarning, message=".*Unable to import Axes3D.*")

# ROS 2 相关库
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data
import tf2_ros

# YOLO 库
try:
    from ultralytics import YOLO, YOLOE
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class PatrolFusionNode(Node):
    def __init__(self, target_classes, model_path):
        super().__init__('patrol_fusion_node')

        # ================= 1. 从 config/patrol_parameters.yaml 动态加载配置 =================
        import yaml
        pkg_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        patrol_yaml_path = os.path.join(pkg_root, 'config', 'patrol_parameters.yaml')
        
        # 默认回退值
        self.ALERT_DIST_THRESHOLD = 10.0  
        self.DETECTION_CONF_THRESHOLD = 0.40
        self.PROCESS_INTERVAL = 1.0  # 1Hz 消费频率
        self.SERVER_IP = "10.21.31.103"
        self.PORT = 30000
        self.out_dir = "alerts"
        self.image_topic = "/left_camera/image"
        self.lidar_topic = "/livox/lidar"
        self.pub_annotated = True
        self.MAX_ALERT_SNAPSHOTS = 200  # 磁盘容量保护：最多个警报快照
        self.CLUSTER_DIST_THRESHOLD = 2.0  # 实例关联距离阈值 (放宽为 2.0 米)

        # 校验真实重定位成功标志
        self.is_localized = False
        self.last_pose_stamp = 0.0

        # 订阅 2D / 3D 定位节点的姿态话题 (确认重定位已被用户成功放置)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.pose_sub_3d = self.create_subscription(
            PoseStamped, '/localization_pose', self.pose_3d_callback, 10)

        # 标注图像与 3D MarkerArray 发布话题
        if self.pub_annotated:
            self.annotated_img_pub = self.create_publisher(Image, '/patrol/annotated_image', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/patrol/target_markers', 10)

        # TF2 监听器 (实时暂存全局机器人位姿)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if os.path.exists(patrol_yaml_path):
            try:
                with open(patrol_yaml_path, 'r') as f:
                    patrol_cfg_raw = yaml.safe_load(f)
                if '/**' in patrol_cfg_raw and 'ros__parameters' in patrol_cfg_raw['/**']:
                    p_data = patrol_cfg_raw['/**']['ros__parameters']
                elif 'ros__parameters' in patrol_cfg_raw:
                    p_data = patrol_cfg_raw['ros__parameters']
                else:
                    p_data = patrol_cfg_raw

                self.ALERT_DIST_THRESHOLD = float(p_data.get('alert_dist_threshold', self.ALERT_DIST_THRESHOLD))
                self.DETECTION_CONF_THRESHOLD = float(p_data.get('detection_conf_threshold', self.DETECTION_CONF_THRESHOLD))
                self.PROCESS_INTERVAL = float(p_data.get('process_interval', self.PROCESS_INTERVAL))
                self.SERVER_IP = str(p_data.get('server_ip', self.SERVER_IP))
                self.PORT = int(p_data.get('server_port', self.PORT))
                self.out_dir = str(p_data.get('output_dir', self.out_dir))
                self.image_topic = str(p_data.get('image_topic', self.image_topic))
                self.lidar_topic = str(p_data.get('lidar_topic', self.lidar_topic))
                self.pub_annotated = bool(p_data.get('publish_annotated_image', self.pub_annotated))
                self.get_logger().info(f"Successfully loaded patrol parameters from {patrol_yaml_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse {patrol_yaml_path}, using default parameters: {e}")

        self.bridge = CvBridge()
        
        if os.path.exists(self.out_dir):
            shutil.rmtree(self.out_dir)
        os.makedirs(self.out_dir, exist_ok=True)
        self.get_logger().info(f"Initialized alert output directory: {self.out_dir}/")

        # 标注图像发布话题
        if self.pub_annotated:
            self.annotated_img_pub = self.create_publisher(Image, '/patrol/annotated_image', 10)

        # ================= 2. 初始化手电筒 UDP 客户端配置 =================
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info(f"Initialized UDP flashlight client, target IP: {self.SERVER_IP}:{self.PORT}")

        # ================= 3. 初始化环境与自动自适应加载模型 =================
        patrol_dir = os.path.dirname(os.path.abspath(__file__))
        clip_model_path = os.path.join(patrol_dir, 'mobileclip2_b.ts')
        cache_dir = os.path.expanduser('~/.cache/ultralytics')
        os.makedirs(cache_dir, exist_ok=True)

        if os.path.exists(clip_model_path):
            target_cache_path = os.path.join(cache_dir, 'mobileclip2_b.ts')
            if not os.path.exists(target_cache_path):
                try:
                    shutil.copy(clip_model_path, target_cache_path)
                except Exception:
                    pass

        # 开放词汇 YOLOE 模型加载 (必须通过 set_classes 注入 CLIP 文本特征)
        pt_model_path = os.path.join(patrol_dir, 'yoloe-26s-seg.pt')
        if not os.path.exists(pt_model_path):
            pt_model_path = model_path

        self.get_logger().info(f"[PATROL] Loading YOLOE open-vocabulary model ({pt_model_path}) for targets: {target_classes} ...")
        self.model = YOLOE(pt_model_path)
        orig_cwd = os.getcwd()
        try:
            os.chdir(patrol_dir)
            self.model.set_classes(target_classes)
            self.get_logger().info(f"[PATROL] Successfully injected target classes into YOLOE: {target_classes}")
        except Exception as e:
            self.get_logger().error(f"Failed to set model classes: {e}")
        finally:
            os.chdir(orig_cwd)

        # ================= 4. 配置相机与雷达内外参 (从 device_parameters.yaml 读取) =================
        import yaml
        pkg_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        yaml_path = os.path.join(pkg_root, 'config', 'device_parameters.yaml')
        
        with open(yaml_path, 'r') as f:
            cfg_raw = yaml.safe_load(f)
            
        if '/**' in cfg_raw and 'ros__parameters' in cfg_raw['/**']:
            cfg_data = cfg_raw['/**']['ros__parameters']
        elif 'ros__parameters' in cfg_raw:
            cfg_data = cfg_raw['ros__parameters']
        else:
            cfg_data = cfg_raw

        cam_cfg = cfg_data['camera']
        self.img_w = int(cam_cfg['width'])
        self.img_h = int(cam_cfg['height'])
        fx = float(cam_cfg['fx'])
        fy = float(cam_cfg['fy'])
        cx = float(cam_cfg['cx'])
        cy = float(cam_cfg['cy'])
        self.K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        
        d0 = float(cam_cfg['d0'])
        d1 = float(cam_cfg['d1'])
        d2 = float(cam_cfg['d2'])
        d3 = float(cam_cfg['d3'])
        self.D = np.array([d0, d1, d2, d3], dtype=np.float64)
        
        ext_cfg = cfg_data['extrin_calib']
        self.T_L2C = np.array(ext_cfg['T_camera_lidar'], dtype=np.float64).reshape(4, 4)
        self.get_logger().info(f"Loaded intrinsics and extrinsics from {yaml_path}")

        self.R_L2C = self.T_L2C[:3, :3]
        self.t_L2C = self.T_L2C[:3, 3]
        self.rvec, _ = cv2.Rodrigues(self.R_L2C)

        # 5. 生产者-消费者解耦数据共享结构 (采用高容错 QoS 防止 DDS 底层静默丢帧)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.data_lock = threading.Lock()
        self.latest_img_msg = None
        self.latest_pcd_msg = None
        self.total_received_imgs = 0
        self.running = True
        
        # 目标实例管理结构
        self.tracked_instances = []
        self.instance_counter = 0

        # 生产者：极速 ROS 2 订阅回调 (零阻塞接收)
        self.img_sub = self.create_subscription(
            Image, self.image_topic, self.img_callback, custom_qos)
        self.pcd_sub = self.create_subscription(
            PointCloud2, self.lidar_topic, self.pcd_callback, qos_profile_sensor_data)
            
        self.get_logger().info(f"Subscribed to '{self.image_topic}' and '{self.lidar_topic}' with Producer-Consumer architecture!")

        # 消费者：启动独立后台 YOLO 推理线程 (1Hz 消费)
        self.consumer_thread = threading.Thread(target=self.infer_worker_loop)
        self.consumer_thread.daemon = True
        self.consumer_thread.start()

    def get_current_robot_pose(self):
        """非阻塞安全查询 map -> base_footprint / body 实时 2D 位姿，自动检测定位激活状态"""
        try:
            target_frame = None
            if self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                target_frame = 'base_footprint'
            elif self.tf_buffer.can_transform('map', 'body', rclpy.time.Time()):
                target_frame = 'body'

            if target_frame is not None:
                transform = self.tf_buffer.lookup_transform('map', target_frame, rclpy.time.Time())
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                yaw = np.arctan2(siny_cosp, cosy_cosp)
                return tx, ty, yaw
            else:
                return None
        except Exception:
            return None

    def pose_callback(self, msg):
        """收到 2D AMCL 重定位姿态反馈，标志重定位已成功激活"""
        self.is_localized = True
        self.last_pose_stamp = time.time()

    def pose_3d_callback(self, msg):
        """收到 3D 室内重定位姿态反馈，标志重定位已成功激活"""
        self.is_localized = True
        self.last_pose_stamp = time.time()

    def img_callback(self, img_msg):
        # 生产者：极速微秒级存指针，决不在回调中做阻塞查询
        with self.data_lock:
            self.latest_img_msg = img_msg
            self.total_received_imgs += 1

    def pcd_callback(self, pcd_msg):
        # 生产者：微秒级只存指针
        with self.data_lock:
            self.latest_pcd_msg = pcd_msg

    def send_flashlight_cmd(self, front_state, back_state, msg_id):
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        cmd_dict = {
            "PatrolDevice": {
                "Type": 1101,
                "Command": 2,
                "Time": current_time,
                "Items": {
                    "Front": front_state,
                    "Back": back_state
                }
            }
        }
        json_bytes = json.dumps(cmd_dict).encode('utf-8')
        header = struct.pack('<4B H H B 7B', 0xeb, 0x91, 0xeb, 0x90, len(json_bytes), msg_id, 0x01, 0, 0, 0, 0, 0, 0, 0)
        try:
            self.udp_sock.sendto(header + json_bytes, (self.SERVER_IP, self.PORT))
        except Exception as e:
            self.get_logger().error(f"Failed to send flashlight UDP command: {e}")

    def trigger_flashlight_blink(self):
        try:
            self.send_flashlight_cmd(front_state=1, back_state=1, msg_id=1)
            time.sleep(0.25)
            self.send_flashlight_cmd(front_state=0, back_state=0, msg_id=2)
            time.sleep(0.25)
            self.send_flashlight_cmd(front_state=1, back_state=1, msg_id=1)
            time.sleep(0.25)
            self.send_flashlight_cmd(front_state=0, back_state=0, msg_id=2)
        except Exception as e:
            self.get_logger().error(f"Flashlight async blink thread exception: {e}")

    def update_target_instance(self, class_name, target_map_x, target_map_y):
        """在线增量均值更新算法：匹配 0.8 米内的已知同类物体，若找到则滑动平均更新，否则新增实例"""
        matched_instance = None
        min_dist = float('inf')

        for inst in self.tracked_instances:
            if inst['class_name'] == class_name:
                dist = np.hypot(target_map_x - inst['x'], target_map_y - inst['y'])
                if dist < self.CLUSTER_DIST_THRESHOLD and dist < min_dist:
                    min_dist = dist
                    matched_instance = inst

        if matched_instance is not None:
            # 在线移动平均 (Online Moving Average) 更新
            count = matched_instance['count'] + 1
            matched_instance['count'] = count
            matched_instance['x'] += (target_map_x - matched_instance['x']) / count
            matched_instance['y'] += (target_map_y - matched_instance['y']) / count
            return matched_instance
        else:
            # 新建实例
            self.instance_counter += 1
            new_inst = {
                'id': self.instance_counter,
                'class_name': class_name,
                'x': target_map_x,
                'y': target_map_y,
                'count': 1
            }
            self.tracked_instances.append(new_inst)
            return new_inst

    def publish_target_markers(self):
        """发布 3D MarkerArray 供 RViz 可视化 (高亮圆柱 + 立体文本)"""
        marker_array = MarkerArray()
        
        for inst in self.tracked_instances:
            # 1. 3D 圆柱/球体 Marker
            shape_marker = Marker()
            shape_marker.header.frame_id = "map"
            shape_marker.header.stamp = self.get_clock().now().to_msg()
            shape_marker.ns = "patrol_target_shapes"
            shape_marker.id = inst['id']
            shape_marker.type = Marker.CYLINDER
            shape_marker.action = Marker.ADD
            shape_marker.pose.position.x = float(inst['x'])
            shape_marker.pose.position.y = float(inst['y'])
            shape_marker.pose.position.z = 0.5
            shape_marker.scale.x = 0.4   # 小巧精致圆点
            shape_marker.scale.y = 0.4
            shape_marker.scale.z = 1.0   # 柱高 1.0 米
            shape_marker.color.r = 1.0
            shape_marker.color.g = 0.0
            shape_marker.color.b = 0.0
            shape_marker.color.a = 0.95
            marker_array.markers.append(shape_marker)

            # 2. 醒目悬浮文字 Marker (2D 俯视角下 Y 轴向上偏移 0.8m，避免覆盖红点)
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "patrol_target_texts"
            text_marker.id = inst['id'] + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(inst['x'])
            text_marker.pose.position.y = float(inst['y']) + 1.2   # Y 轴拉开 1.2m 避开点位
            text_marker.pose.position.z = 1.0
            text_marker.scale.z = 0.8                              # 醒目大字
            text_marker.color.r = 1.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0                              # 纯鲜红色 (Red)
            text_marker.color.a = 1.0
            text_marker.text = f"#{inst['id']}: {inst['class_name']} ({inst['count']})"
            marker_array.markers.append(text_marker)

        if marker_array.markers and rclpy.ok():
            try:
                self.marker_pub.publish(marker_array)
            except Exception:
                pass

    # ================= 消费者核心循环 (独立线程) =================
    def infer_worker_loop(self):
        last_processed_sec = 0.0
        self.get_logger().info("Consumer inference worker thread started successfully!")
        
        while self.running and rclpy.ok():
            time.sleep(0.05)  # 20Hz 轮询检查
            with self.data_lock:
                img_msg = self.latest_img_msg
                pcd_msg = self.latest_pcd_msg
                total_recv = self.total_received_imgs

            if img_msg is None:
                continue

            msg_sec = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9

            # 兼容 rosbag 寻道或重播
            if msg_sec < last_processed_sec or (msg_sec - last_processed_sec > 10.0):
                last_processed_sec = 0.0

            # 严格按照 1.0s 图像时间跨度控制消费
            if msg_sec - last_processed_sec < self.PROCESS_INTERVAL:
                continue

            last_processed_sec = msg_sec
            timestamp_str = f"{msg_sec:.3f}"
            process_start = time.time()

            # 非阻塞动态查询 2D 重定位 TF 姿态
            robot_pose = self.get_current_robot_pose()

            if robot_pose is None:
                self.get_logger().info("Standby: Waiting for 2D relocalization...", throttle_duration_sec=3.0)
                continue

            try:
                img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8").copy()
            except Exception as e:
                self.get_logger().error(f"Image conversion failed: {e}")
                continue

            if self.model is None:
                continue

            # 1. YOLO 目标检测 (耗时 100~170ms)
            results = self.model.predict(source=img, conf=self.DETECTION_CONF_THRESHOLD, verbose=False)
            boxes = results[0].boxes
            
            target_detected = False
            valid_depths = []
            points_2d = []

            if len(boxes) > 0:
                if pcd_msg is not None:
                    point_gen = point_cloud2.read_points(pcd_msg, field_names=("x", "y", "z"), skip_nans=True)
                    struct_points = np.array(list(point_gen))
                    
                    # 算力优化：对点云进行 3 倍步长切片采样，保留约 8000 个有效点，测距精度不变但 CPU 投影开销降低 66%
                    if struct_points.shape[0] > 0:
                        struct_points = struct_points[::3]
                        points_3d = np.column_stack((
                            struct_points['x'], 
                            struct_points['y'], 
                            struct_points['z']
                        )).astype(np.float64)
                        
                        proj_pts, _ = cv2.projectPoints(points_3d, self.rvec, self.t_L2C, self.K, self.D)
                        points_2d = proj_pts.reshape(-1, 2)
                        
                        points_c = (self.R_L2C @ points_3d.T).T + self.t_L2C
                        distances = np.linalg.norm(points_c, axis=1) 
                        
                        valid_mask = (points_c[:, 2] > 0) & \
                                     (points_2d[:, 0] >= 0) & (points_2d[:, 0] < self.img_w) & \
                                     (points_2d[:, 1] >= 0) & (points_2d[:, 1] < self.img_h)
                        
                        points_2d = points_2d[valid_mask]
                        valid_depths = distances[valid_mask]

            alert_triggered = False
            fx = self.K[0, 0]
            cx = self.K[0, 2]

            for box in boxes:
                cls_id = int(box.cls)
                class_name = self.model.names[cls_id] if hasattr(self.model, 'names') else str(cls_id)
                confidence = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                target_dist = -1.0
                if len(points_2d) > 0:
                    w, h = x2 - x1, y2 - y1
                    cx1, cx2 = x1 + w * 0.25, x2 - w * 0.25
                    cy1, cy2 = y1 + h * 0.25, y2 - h * 0.25
                    
                    in_box = (points_2d[:, 0] >= cx1) & (points_2d[:, 0] <= cx2) & \
                             (points_2d[:, 1] >= cy1) & (points_2d[:, 1] <= cy2)
                    
                    box_depths = valid_depths[in_box]
                    if len(box_depths) > 0:
                        target_dist = np.median(box_depths)

                # ================= 目标 2D 全局地图聚类与在线均值更新 =================
                inst_info_str = ""
                if target_dist > 0 and robot_pose is not None:
                    rx, ry, ryaw = robot_pose
                    box_center_x = (x1 + x2) / 2.0
                    # 相机光轴水平偏角 (弧度)
                    angle_offset = np.arctan2(box_center_x - cx, fx)
                    # 全局物理视角 (相机坐标系 Z 轴向右 X 向前)
                    obj_global_angle = ryaw - angle_offset
                    
                    obj_map_x = rx + target_dist * np.cos(obj_global_angle)
                    obj_map_y = ry + target_dist * np.sin(obj_global_angle)

                    inst = self.update_target_instance(class_name, obj_map_x, obj_map_y)
                    inst_info_str = f" [Inst #{inst['id']} (x={inst['x']:.2f}, y={inst['y']:.2f}, N={inst['count']})]"

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.6
                thickness = 2
                
                if 0 < target_dist < self.ALERT_DIST_THRESHOLD:
                    alert_triggered = True
                    label = f"{class_name}:{target_dist:.2f}m conf:{confidence:.2f} !!ALERT!!"
                    color = (0, 0, 255)
                    self.get_logger().warn(f"WARNING: Dangerous object '{class_name}' detected at {timestamp_str}s, distance: {target_dist:.2f}m!{inst_info_str}")
                else:
                    distance_label = 'No Data' if target_dist < 0 else f'{target_dist:.2f}m'
                    label = f"{class_name}:{distance_label} conf:{confidence:.2f}"
                    color = (0, 255, 0)
                    
                (text_w, text_h), _ = cv2.getTextSize(label, font, font_scale, thickness)
                text_x = x1 if (x1 + text_w) < self.img_w else max(0, self.img_w - text_w - 5)
                text_y = max(y1 - 10, text_h + 5) if y1 > 30 else y1 + text_h + 5
                cv2.rectangle(img, (text_x, text_y - text_h - 2), (text_x + text_w, text_y + 2), (0, 0, 0), -1)
                cv2.putText(img, label, (text_x, text_y), font, font_scale, color, thickness)

            # 实时发布 RViz 3D Marker 标记点
            self.publish_target_markers()

            # 精简诊断日志：一行展示机器人位姿、检测到的目标数与耗时
            cost_time = time.time() - process_start
            if rclpy.ok():
                self.get_logger().info(f"Pose:({robot_pose[0]:.2f}, {robot_pose[1]:.2f}) | Targets:{len(boxes)} | Cost:{cost_time:.3f}s")

            # 发布标注图像话题
            if hasattr(self, 'annotated_img_pub') and rclpy.ok():
                try:
                    img_c = np.ascontiguousarray(img)
                    msg_out = Image()
                    msg_out.header.stamp = img_msg.header.stamp
                    msg_out.header.frame_id = img_msg.header.frame_id
                    msg_out.height = img_c.shape[0]
                    msg_out.width = img_c.shape[1]
                    msg_out.encoding = "bgr8"
                    msg_out.is_bigendian = 0
                    msg_out.step = img_c.shape[1] * 3
                    msg_out.data = img_c.tobytes()
                    if rclpy.ok():
                        self.annotated_img_pub.publish(msg_out)
                except Exception:
                    pass

            # 抓拍图片并异步控制手电筒 (附带磁盘上限保护)
            if alert_triggered:
                # 检查输出文件夹图片数量，多于 MAX_ALERT_SNAPSHOTS 时清理最早的
                try:
                    existing_files = sorted([os.path.join(self.out_dir, f) for f in os.listdir(self.out_dir) if f.startswith("alert_")], key=os.path.getmtime)
                    if len(existing_files) >= self.MAX_ALERT_SNAPSHOTS:
                        for old_file in existing_files[:len(existing_files) - self.MAX_ALERT_SNAPSHOTS + 1]:
                            os.remove(old_file)
                except Exception:
                    pass

                save_path = os.path.join(self.out_dir, f"alert_{timestamp_str}.png")
                cv2.imwrite(save_path, img)
                
                blink_thread = threading.Thread(target=self.trigger_flashlight_blink)
                blink_thread.daemon = True
                blink_thread.start()


def main(args=None):
    rclpy.init(args=args)
    
    # 动态解析 CLI 或环境变量中的 target_classes
    default_model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "yoloe-26s-seg.pt")
    MODEL_PATH = os.environ.get("YOLO_MODEL_PATH", default_model_path)
    
    # 尝试从环境变量或者系统参数获取目标列表
    env_classes = os.environ.get("PATROL_TARGET_CLASSES", "")
    target_classes = []
    if env_classes:
        target_classes = [t.strip() for t in env_classes.split(',') if t.strip()]
    
    # 临时创建节点来读取从 ros2 launch/cli 传入的 ROS2 参数
    temp_node = Node('patrol_param_reader')
    temp_node.declare_parameter('target_classes', ['fire extinguisher', 'person'])
    ros_param_classes = list(temp_node.get_parameter('target_classes').get_parameter_value().string_array_value)
    temp_node.destroy_node()

    if not target_classes and ros_param_classes:
        target_classes = ros_param_classes

    node = PatrolFusionNode(target_classes, MODEL_PATH)
    
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, Exception) as e:
        print("Patrol node shutdown cleanly.")
    finally:
        node.running = False
        try:
            node.udp_sock.close()
        except Exception:
            pass
        if rclpy.ok():
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == "__main__":
    main()
