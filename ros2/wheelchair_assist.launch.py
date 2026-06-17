#!/usr/bin/env python3
"""
wheelchair_assist.launch.py — sobe o pipeline semiassistido completo:

    [RPLIDAR] --/scan--> [shared_control] --/cmd_vel--> [esp_bridge] --serial--> [ESP32]
       (sllidar_ros2)          ^---/joystick_cmd_vel-----------'

Pré-requisitos:
  sudo apt install ros-jazzy-sllidar-ros2     # ou clone Slamtec/sllidar_ros2 no ws
  Os nós deste projeto (esp_bridge, shared_control) instalados no seu pacote.

LIDAR deste projeto: RPLIDAR C1
  baud 460800, launch dedicado: sllidar_c1_launch.py
  DTOF, alcance 12 m, zona cega 0,05 m, 10 Hz, resolução angular 0,72°.

Uso:
  ros2 launch <seu_pacote> wheelchair_assist.launch.py \
      esp_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0 lidar_baud:=460800
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    esp_port = LaunchConfiguration("esp_port")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baud")

    return LaunchDescription([
        DeclareLaunchArgument("esp_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_baud", default_value="460800"),  # RPLIDAR C1

        # --- Driver do RPLIDAR (sllidar_ros2) ---
        # Publica sensor_msgs/LaserScan em /scan, frame_id 'laser'.
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            parameters=[{
                "serial_port": lidar_port,
                "serial_baudrate": lidar_baud,   # C1 = 460800
                "frame_id": "laser",
                "angle_compensate": True,
                "scan_mode": "Standard",         # modo padrão do C1
            }],
        ),

        # --- Ponte serial com o ESP32 ---
        Node(
            package="wheelchair_ros",          # << troque pelo nome do seu pacote
            executable="esp_bridge",
            name="esp_bridge",
            output="screen",
            parameters=[{
                "port": esp_port,
                "baud": 115200,
                "max_duty": 0.30,              # gate de segurança (== GUI)
                "cmd_rate_hz": 20.0,
            }],
        ),

        # --- Controle compartilhado (semiassistido) ---
        Node(
            package="wheelchair_ros",          # << troque pelo nome do seu pacote
            executable="shared_control",
            name="shared_control",
            output="screen",
            parameters=[{
                "stop_distance": 0.45,
                "slow_distance": 1.10,
                "max_deviation_deg": 45.0,
                "assist_gain": 0.8,
            }],
        ),
    ])
