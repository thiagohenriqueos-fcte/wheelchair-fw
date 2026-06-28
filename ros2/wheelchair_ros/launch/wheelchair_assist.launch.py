from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    esp_port = LaunchConfiguration("esp_port")
    esp_baud = LaunchConfiguration("esp_baud")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")
    armed = LaunchConfiguration("armed")
    max_duty = LaunchConfiguration("max_duty")
    assist_gain = LaunchConfiguration("assist_gain")
    stop_distance = LaunchConfiguration("stop_distance")
    slow_distance = LaunchConfiguration("slow_distance")

    sllidar_launch = PathJoinSubstitution([
        FindPackageShare("sllidar_ros2"),
        "launch",
        "sllidar_c1_launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("esp_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("esp_baud", default_value="115200"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_baud", default_value="460800"),
        DeclareLaunchArgument("armed", default_value="false"),
        DeclareLaunchArgument("max_duty", default_value="0.30"),
        DeclareLaunchArgument("assist_gain", default_value="0.0"),
        DeclareLaunchArgument("stop_distance", default_value="0.45"),
        DeclareLaunchArgument("slow_distance", default_value="1.10"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            launch_arguments={
                "serial_port": lidar_port,
                "serial_baudrate": lidar_baud,
            }.items(),
        ),

        Node(
            package="wheelchair_ros",
            executable="shared_control",
            name="shared_control",
            output="screen",
            parameters=[{
                "assist_gain": assist_gain,
                "stop_distance": stop_distance,
                "slow_distance": slow_distance,
            }],
        ),

        Node(
            package="wheelchair_ros",
            executable="esp_bridge",
            name="esp_bridge",
            output="screen",
            parameters=[{
                "port": esp_port,
                "baud": esp_baud,
                "armed": armed,
                "max_duty": max_duty,
            }],
        ),
    ])
