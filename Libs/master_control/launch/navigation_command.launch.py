from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    require_navigation_subscriber = LaunchConfiguration(
        "require_navigation_subscriber"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "require_navigation_subscriber", default_value="true"
            ),
            Node(
                package="master_control",
                executable="navigation_command_node",
                name="navigation_command_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "require_navigation_subscriber": require_navigation_subscriber,
                    }
                ],
            ),
        ]
    )
