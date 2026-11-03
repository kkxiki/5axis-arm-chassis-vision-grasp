from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    package_share = get_package_share_directory("zhenshi_driver_py")
    default_config = os.path.join(package_share, "config", "driver.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument(
                "robot_ip", default_value="192.168.11.11"
            ),
            DeclareLaunchArgument("dry_run", default_value="true"),
            DeclareLaunchArgument("sdk_root", default_value=""),
            DeclareLaunchArgument("model_index", default_value="0"),
            DeclareLaunchArgument("model_count", default_value="2"),
            Node(
                package="zhenshi_driver_py",
                executable="driver_node",
                name="zhenshi_driver",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "robot_ip": LaunchConfiguration("robot_ip"),
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"),
                            value_type=bool,
                        ),
                        "sdk_root": LaunchConfiguration("sdk_root"),
                        "model_index": ParameterValue(
                            LaunchConfiguration("model_index"),
                            value_type=int,
                        ),
                        "model_count": ParameterValue(
                            LaunchConfiguration("model_count"),
                            value_type=int,
                        ),
                    },
                ],
            ),
        ]
    )
