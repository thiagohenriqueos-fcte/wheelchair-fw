"""
Bringup completo da cadeira de rodas — fase 1 (telemetria + fusao).

Sobe, na ordem:
  1. robot_state_publisher (publica TF a partir do URDF)
  2. wheelchair_bridge    (le o ESP32 e publica /wheel/odom, /joy, /wheel/telemetry)
  3. witmotion_ros        (IMU IWT603T em /imu_data, a ~250 Hz)
  4. sllidar_ros2         (RPLIDAR A1M8 em /scan, ~10 Hz, frame base_laser)
  5. robot_localization   (EKF: funde /wheel/odom + /imu_data -> /odometry/filtered + TF odom->base_link)

Componentes nao incluidos NESTA fase (entram depois):
  - supervisor de seguranca do LiDAR (zonas de parada/reducao)
  - canal Pi -> ESP32 de restricoes de velocidade
  - GUI / dashboard

Use:
    ros2 launch wheelchair_base bringup.launch.py
    ros2 launch wheelchair_base bringup.launch.py rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('wheelchair_base')
    urdf = os.path.join(pkg, 'urdf', 'wheelchair.urdf.xacro')
    bridge_yaml = os.path.join(pkg, 'config', 'wheelchair.yaml')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf.yaml')
    slam_yaml = os.path.join(pkg, 'config', 'slam.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Sobe o RViz junto ao bringup')

    slam_arg = DeclareLaunchArgument(
        'slam', default_value='true',
        description='Sobe o slam_toolbox (pose por LIDAR, frame map)')

    laser_odom_arg = DeclareLaunchArgument(
        'laser_odom', default_value='true',
        description='Sobe o rf2o (odometria a laser -> /odom_rf2o p/ o EKF)')

    lidar_baud_arg = DeclareLaunchArgument(
        'lidar_baud', default_value='115200',
        description='Baudrate do LIDAR (A1M8: 115200, C1: 460800)')

    # 1) robot_state_publisher: TF estatico base_link <-> imu_link <-> base_laser
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', urdf]), value_type=str),
        }],
        output='screen',
    )

    # 2) Ponte serial ESP32 -> ROS
    bridge = Node(
        package='wheelchair_base',
        executable='bridge_node',
        name='wheelchair_bridge',
        parameters=[bridge_yaml],
        output='screen',
        emulate_tty=True,
    )

    # 3) IMU IWT603T — usamos o launch wt61c.launch.py do witmotion_ros.
    #    Voce ja validou esse caminho; aqui apenas chamamos o no
    #    diretamente para manter um unico arquivo de launch coeso.
    #    Os parametros vem do YAML do proprio witmotion_ros instalado;
    #    confira que la consta port: wheelchair/imu (e nao ttyACM0).
    imu = Node(
        package='witmotion_ros',
        executable='witmotion_ros_node',
        name='witmotion',
        parameters=[os.path.join(
            get_package_share_directory('witmotion_ros'),
            'config', 'wt61c.yml')],
        output='screen',
    )

    # 4) RPLIDAR A1M8 @ 115200 ou C1 @ 460800 — pacote oficial Slamtec (sllidar_ros2).
    #    O default do driver e frame_id "laser"; sobrescrevemos para
    #    "base_laser" para casar com o URDF. Baudrate parametrizável via arg.
    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/wheelchair/lidar',
            'serial_baudrate': LaunchConfiguration('lidar_baud'),
            'frame_id': 'base_laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        output='screen',
    )

    # 4b) rf2o — odometria a laser (scan-matching). Sem encoders, e ela que da
    #     a translacao (vx) que falta ao EKF. publish_tf=false: quem publica
    #     odom->base_link e o EKF (evita TF duplicada).
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        condition=IfCondition(LaunchConfiguration('laser_odom')),
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

    # 5) EKF (robot_localization) — funde wheel/odom + rf2o + imu/data
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_yaml],
        output='screen',
    )

    # 6) slam_toolbox — pose por LIDAR (scan-matching + loop closure).
    #    Publica TF map->odom, corrigindo a deriva do EKF (que, sem encoders,
    #    e essencialmente so-IMU). A GUI le map->base_link.
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_yaml],
        condition=IfCondition(LaunchConfiguration('slam')),
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        rviz_arg,
        slam_arg,
        laser_odom_arg,
        lidar_baud_arg,
        rsp,
        bridge,
        imu,
        lidar,
        rf2o,
        ekf,
        slam,
        rviz,
    ])
