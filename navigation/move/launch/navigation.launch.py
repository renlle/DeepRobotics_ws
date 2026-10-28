import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('move')
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    
    # 动态推导数据路径，使地图和参数能够跨设备自适应
    data_dir = os.path.join(ws_root, 'src', 'DeepRobotics_ws', 'data')
    if not os.path.exists(data_dir):
        data_dir = os.path.join(ws_root, 'src', 'data')
    if not os.path.exists(data_dir):
        data_dir = os.path.join(ws_root, 'data')
        
    default_map_yaml = os.path.join(data_dir, 'map_2d.yaml')
    default_map_pcd = os.path.join(data_dir, 'map_3d.pcd')
    # 按环境后缀选择配置，同时保留原有工作区路径回退顺序。
    env_name_szs = os.environ.get('ENV_NAME_SZS', '')
    nav_params_name = f'nav_parameters{env_name_szs}.yaml'
    default_params_file = os.path.join(ws_root, 'src', 'DeepRobotics_ws', 'config', nav_params_name)
    if not os.path.exists(default_params_file):
        default_params_file = os.path.join(ws_root, 'src', 'config', nav_params_name)
    if not os.path.exists(default_params_file):
        default_params_file = os.path.join(ws_root, 'config', nav_params_name)

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_pcd = LaunchConfiguration("map_pcd")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    start_external_nav = LaunchConfiguration("start_external_nav")
    start_rviz_waypoints = LaunchConfiguration("start_rviz_waypoints")
    clicked_point_topic = LaunchConfiguration("clicked_point_topic")
    waypoint_delete_topic = LaunchConfiguration("waypoint_delete_topic")
    waypoint_replace_topic = LaunchConfiguration("waypoint_replace_topic")
    waypoint_status_topic = LaunchConfiguration("waypoint_status_topic")
    waypoint_replan_period = LaunchConfiguration("waypoint_replan_period")
    waypoint_goal_tolerance = LaunchConfiguration("waypoint_goal_tolerance")
    waypoint_edit_radius = LaunchConfiguration("waypoint_edit_radius")
    waypoint_enable_interactive_markers = LaunchConfiguration("waypoint_enable_interactive_markers")
    waypoint_interactive_marker_ns = LaunchConfiguration("waypoint_interactive_marker_ns")
    adapter_path_timeout = LaunchConfiguration("adapter_path_timeout")

    use_map_server = LaunchConfiguration("use_map_server")

    # 1. 2D 网格地图服务器节点
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": ["map_server"]},
            {"bond_timeout": 0.0},
        ],
    )
    
    map_server_group = GroupAction(
        condition=IfCondition(use_map_server),
        actions=[
            TimerAction(period=3.0, actions=[map_server, map_lifecycle])
        ]
    )

    nav2_navigation = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("move"), "launch", "nav_core.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": autostart,
                }.items(),
            ),
        ]
    )

    # 3. 引入外部控制管道（包含纯追踪、RL局部规划器、与控制命令适配器）
    external_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("move"), "launch", "priest_external_nav.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_rviz_waypoints": start_rviz_waypoints,
            "clicked_point_topic": clicked_point_topic,
            "waypoint_delete_topic": waypoint_delete_topic,
            "waypoint_replace_topic": waypoint_replace_topic,
            "waypoint_status_topic": waypoint_status_topic,
            "waypoint_replan_period": waypoint_replan_period,
            "waypoint_goal_tolerance": waypoint_goal_tolerance,
            "waypoint_edit_radius": waypoint_edit_radius,
            "waypoint_enable_interactive_markers": waypoint_enable_interactive_markers,
            "waypoint_interactive_marker_ns": waypoint_interactive_marker_ns,
            "global_path_topic": "global_path",
            "waypoint_path_topic": "waypoint_global_path",
            "pure_pursuit_plan_topic": "execution_global_path",
            "subgoal_topic": "subgoal",
            "final_goal_topic": "final_goal",
            "local_path_topic": "local_path",
            "cmd_vel_topic": "/cmd_vel",
            "nav_cmd_topic": "/NAV_CMD",
            "global_frame": "map",
            "robot_frame": "base_footprint",
            "odom_frame": "camera_init_footprint",
            "path_target_frame": "camera_init_footprint",
            "adapter_path_timeout": adapter_path_timeout,
        }.items(),
    )

    external_nav_group = GroupAction(
        condition=IfCondition(start_external_nav),
        actions=[TimerAction(period=11.0, actions=[external_nav])],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "map_pcd",
                default_value=default_map_pcd,
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map_yaml,
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
            ),
            DeclareLaunchArgument("start_external_nav", default_value="true"),
            DeclareLaunchArgument("start_rviz_waypoints", default_value="true"),
            DeclareLaunchArgument("clicked_point_topic", default_value="/clicked_point"),
            DeclareLaunchArgument("waypoint_delete_topic", default_value="/waypoint_sequence/delete_nearest"),
            DeclareLaunchArgument("waypoint_replace_topic", default_value="/waypoint_sequence/replace_nearest"),
            DeclareLaunchArgument("waypoint_status_topic", default_value="/waypoint_sequence/status"),
            DeclareLaunchArgument("waypoint_replan_period", default_value="1.0"),
            DeclareLaunchArgument("waypoint_goal_tolerance", default_value="1.2"),
            DeclareLaunchArgument("waypoint_edit_radius", default_value="1.5"),
            DeclareLaunchArgument("waypoint_enable_interactive_markers", default_value="true"),
            DeclareLaunchArgument("waypoint_interactive_marker_ns", default_value="waypoint_editor"),
            DeclareLaunchArgument("use_map_server", default_value="false", description="Whether to launch map_server in navigation (set false if localization already provides it)"),
            DeclareLaunchArgument("adapter_path_timeout", default_value="1.2"),

            map_server_group,
            TimerAction(period=8.0, actions=[nav2_navigation]),
            external_nav_group,

        ]
    )
