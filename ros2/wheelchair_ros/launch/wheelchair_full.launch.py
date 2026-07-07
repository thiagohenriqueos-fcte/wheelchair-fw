"""Stack completo da cadeira para rodar headless (no boot, sem GUI).

Camada SEMPRE ativa (supervisor de segurança semi-assistido):
  - robot_state_publisher  (TF do URDF: base_link <-> base_laser/imu_link)
  - sllidar_node           (RPLIDAR -> /scan)
  - esp_bridge             (ESP <-> ROS: /joystick_cmd_vel, /cmd_vel -> drive_cmd)
  - shared_control         (função custo sobre /scan -> /cmd_vel)
  - witmotion              (IMU -> /imu_data)        [imu:=true]

Camada de NAVEGAÇÃO (opcional, pesada — use hub USB alimentado):  [nav:=true]
  - rf2o_laser_odometry    (odometria a laser -> /odom_rf2o)
  - ekf_filter_node        (funde rf2o + imu -> /odometry/filtered + TF odom)
  - slam_toolbox           (pose corrigida -> TF map->odom)

A esp_bridge é a ÚNICA dona da porta serial do ESP (a antiga
wheelchair_base/bridge_node não entra aqui: é incompatível com o firmware
JSON atual e disputaria a mesma porta).

Validado em bancada (2026-07): supervisor leve (nav:=false) sobe com LIDAR a
~10 Hz e ~4,6 GiB livres na Pi 5; a camada nav é a que pesa.

Exemplos:
  # supervisor leve (recomendado p/ validar primeiro, não satura a Pi):
  ros2 launch wheelchair_ros wheelchair_full.launch.py nav:=false armed:=false

  # stack completo, pronto p/ dirigir:
  ros2 launch wheelchair_ros wheelchair_full.launch.py nav:=true armed:=true \
       assist_gain:=0.8
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    base = get_package_share_directory('wheelchair_base')
    urdf = os.path.join(base, 'urdf', 'wheelchair.urdf.xacro')
    ekf_yaml = os.path.join(base, 'config', 'ekf.yaml')
    slam_yaml = os.path.join(base, 'config', 'slam.yaml')
    imu_yaml = os.path.join(
        get_package_share_directory('witmotion_ros'), 'config', 'wt61c.yml')

    nav = LaunchConfiguration('nav')
    imu = LaunchConfiguration('imu')

    args = [
        DeclareLaunchArgument('nav', default_value='true',
                              description='Sobe rf2o + EKF + slam_toolbox'),
        DeclareLaunchArgument('imu', default_value='true',
                              description='Sobe o nó do IMU WitMotion'),
        DeclareLaunchArgument('armed', default_value='false',
                              description='Arma a ponte (envia movimento). '
                                          'Mantenha false até validar.'),
        DeclareLaunchArgument('max_duty', default_value='0.20'),
        DeclareLaunchArgument('assist_gain', default_value='0.0',
                              description='Ganho de desvio (0 = só freio/parada)'),
        DeclareLaunchArgument('stop_distance', default_value='0.45'),
        DeclareLaunchArgument('slow_distance', default_value='1.10'),
        # Calibrado em bancada p/ esta montagem do LIDAR (frente do chassi ~149°
        # no referencial do sensor). A estrutura da cadeira fica fora do cone.
        DeclareLaunchArgument('front_offset_deg', default_value='149.0'),
        DeclareLaunchArgument('cone_half_deg', default_value='20.0'),
        DeclareLaunchArgument('min_obstacle_range', default_value='0.20'),
        DeclareLaunchArgument('esp_port', default_value='/dev/wheelchair/esp32'),
        DeclareLaunchArgument('esp_baud', default_value='460800'),
        # Odometria por encoder (alimenta o EKF). Calibrado em bancada:
        # wheel_radius por empurrão de 3 m; wheel_base por 3 voltas (1080°).
        DeclareLaunchArgument('wheel_radius', default_value='0.293'),
        DeclareLaunchArgument('wheel_base', default_value='0.522'),
        # Com o firmware fw 0.9.x (encoder esquerdo nos GPIO 18/19), ambas as
        # rodas contam NEGATIVO para frente -> invertemos os dois (bancada).
        DeclareLaunchArgument('enc_left_sign', default_value='-1'),
        DeclareLaunchArgument('enc_right_sign', default_value='-1'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/wheelchair/lidar'),
        DeclareLaunchArgument('lidar_baud', default_value='460800',
                              description='C1: 460800, A1M8: 115200'),
        DeclareLaunchArgument('imu_port', default_value='/dev/wheelchair/imu'),
    ]

    # ── TF do robô ───────────────────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description':
                ParameterValue(Command(['xacro ', urdf]), value_type=str),
        }],
        output='screen',
    )

    # ── LIDAR ────────────────────────────────────────────────────────────────
    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baud'),
            'frame_id': 'base_laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        output='screen',
    )

    # ── Ponte serial do ESP (única dona da porta) ────────────────────────────
    esp_bridge = Node(
        package='wheelchair_ros',
        executable='esp_bridge',
        name='esp_bridge',
        parameters=[{
            'port': LaunchConfiguration('esp_port'),
            'baud': LaunchConfiguration('esp_baud'),
            'armed': LaunchConfiguration('armed'),
            'max_duty': LaunchConfiguration('max_duty'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'enc_left_sign': LaunchConfiguration('enc_left_sign'),
            'enc_right_sign': LaunchConfiguration('enc_right_sign'),
        }],
        output='screen',
    )

    # ── Supervisor de segurança (função custo) ───────────────────────────────
    shared_control = Node(
        package='wheelchair_ros',
        executable='shared_control',
        name='shared_control',
        parameters=[{
            'assist_gain': LaunchConfiguration('assist_gain'),
            'stop_distance': LaunchConfiguration('stop_distance'),
            'slow_distance': LaunchConfiguration('slow_distance'),
            'front_offset_deg': LaunchConfiguration('front_offset_deg'),
            'cone_half_deg': LaunchConfiguration('cone_half_deg'),
            'min_obstacle_range': LaunchConfiguration('min_obstacle_range'),
        }],
        output='screen',
    )

    # ── IMU ──────────────────────────────────────────────────────────────────
    witmotion = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        name='witmotion',
        condition=IfCondition(imu),
        parameters=[imu_yaml, {'port': LaunchConfiguration('imu_port')}],
        output='screen',
    )

    # ── Navegação (opcional, pesada) ─────────────────────────────────────────
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        condition=IfCondition(nav),
        # rf2o loga a pose em nível INFO a cada ciclo (~10 Hz), o que inunda o
        # journal no boot; sobe o nível para WARN.
        arguments=['--ros-args', '--log-level', 'rf2o_laser_odometry:=warn'],
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        condition=IfCondition(nav),
        parameters=[ekf_yaml],
        output='screen',
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        condition=IfCondition(nav),
        parameters=[slam_yaml],
        output='screen',
    )

    return LaunchDescription(args + [
        rsp, lidar, esp_bridge, shared_control, witmotion, rf2o, ekf, slam,
    ])
