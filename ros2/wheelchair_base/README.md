# wheelchair_base

Pacote ROS2 (Jazzy / ament_python) que integra a cadeira de rodas semiassistida:
ESP32 (controle local + encoders + joystick) -> Raspberry Pi 5 (ROS2 + IMU IWT603T + RPLIDAR A1M8).

**Fase atual: apenas TELEMETRIA.** O ESP32 envia para o Pi; o Pi nao envia
nada de volta. O loop de controle continua 100% local no firmware, como previsto
no documento de arquitetura. O canal de restricoes de velocidade e o supervisor
de seguranca do LiDAR entram em fase posterior.

## O que entra no ar

| No | Topico/saida | Taxa |
|---|---|---|
| `wheelchair_bridge` | `/wheel/odom` (`nav_msgs/Odometry`), `/joy`, `/wheel/telemetry` | ~20 Hz |
| `witmotion` (witmotion_ros) | `/imu_data` (`sensor_msgs/Imu`) | ~250 Hz |
| `sllidar_node` | `/scan` (`sensor_msgs/LaserScan`) | ~10 Hz |
| `ekf_filter_node` (robot_localization) | `/odometry/filtered` + TF `odom -> base_link` | 30 Hz |
| `robot_state_publisher` | TF estaticos `base_link <-> imu_link <-> base_laser` | — |

## Instalacao

```bash
cd ~/dev_ws/src
# coloque este pacote aqui (wheelchair_base/)

# 1) regras udev — symlinks estaveis para os tres seriais
sudo cp wheelchair_base/udev/99-wheelchair.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/wheelchair/   # deve mostrar imu, esp32 e lidar

# 2) dependencias ROS
sudo apt install ros-jazzy-robot-localization ros-jazzy-robot-state-publisher \
                 ros-jazzy-xacro ros-jazzy-sllidar-ros2

# 3) build
cd ~/dev_ws
colcon build --packages-select wheelchair_base --symlink-install
source install/setup.bash
```

Tambem ajuste, uma vez, o `wt61c.yml` instalado em
`~/dev_ws/install/witmotion_ros/share/witmotion_ros/config/` para
`port: wheelchair/imu` (em vez de `ttyACM0`), para sobreviver a re-enumeracoes.

## Uso

```bash
ros2 launch wheelchair_base bringup.launch.py            # tudo, sem RViz
ros2 launch wheelchair_base bringup.launch.py rviz:=true # com RViz
```

Em outro terminal, verificacoes rapidas:

```bash
ros2 topic hz /imu_data /wheel/odom /scan /odometry/filtered
ros2 run tf2_tools view_frames    # gera frames.pdf — confira a TF tree
ros2 run rqt_tf_tree rqt_tf_tree  # ou visualmente
```

## Calibracao indispensavel antes de fundir

1. **`wheel_base`** (em `config/wheelchair.yaml`): gire 360 graus no lugar com
   `/wheel/odom` rodando; o yaw integrado tem que bater com uma volta real.
   Ajuste ate fechar.
2. **PPR/diametro da roda** (no firmware do ESP32): ande 1 m em linha reta
   medindo com trena; ajuste `WHEEL_DIAMETER_M` ou `ENCODER_PPR` ate `D`
   bater com 1.00 m.
3. **Orientacao do IMU** (no `urdf/wheelchair.urdf.xacro`): pare a cadeira,
   aponte +X do robo para frente, leia `/imu_data` — o eixo X do acelerometro
   deve ser proximo de zero (a gravidade so aparece em Z). Se sair errado,
   ajuste o `rpy` do joint `base_link_to_imu`.
4. **Montagem do LiDAR** (mesmo URDF): meca a altura real e o offset
   frontal; ajuste `lidar_x` e `lidar_z`. Se o RPLIDAR estiver de cabeca
   para baixo, ajuste `rpy="3.14159 0 0"`.

## Estrutura

```
wheelchair_base/
  package.xml
  setup.py
  setup.cfg
  wheelchair_base/
    __init__.py
    bridge_node.py        # ponte serial ESP32 -> ROS
  launch/
    bringup.launch.py     # sobe todos os nos
  config/
    wheelchair.yaml       # parametros da ponte
    ekf.yaml              # robot_localization
  urdf/
    wheelchair.urdf.xacro # frames base_link/imu_link/base_laser
  udev/
    99-wheelchair.rules
```

## Proxima fase (NAO incluida)

- Canal `/wheelchair/speed_limits` (`std_msgs/Float32MultiArray`) do Pi para
  o ESP32, com tetos por direcao (fwd/rev/yaw_l/yaw_r) renovados a ~10 Hz.
- No supervisor que assina `/scan`, calcula tetos por setor e publica em
  `/wheelchair/speed_limits`.
- Extensao do firmware: aplicar os tetos como saturacao sobre o joystick,
  com expiracao de ~300 ms para um default conservador.

## Onde este pacote vive (importante ao reinstalar)

A fonte de verdade deste pacote e **este repositorio**
(`wheelchair-fw/ros2/wheelchair_base`). Em `~/dev_ws/src/` existe apenas um
symlink apontando para ca:

    ln -s ~/wheelchair-fw/ros2/wheelchair_base ~/dev_ws/src/wheelchair_base

O `colcon` segue o symlink normalmente. Antes disso o pacote vivia solto em
`~/dev_ws/src/`, fora de qualquer git -- e a calibracao do LiDAR no URDF se
perderia numa reinstalacao da Pi.

### Geometria do LiDAR no URDF

`urdf/wheelchair.urdf.xacro` carrega a pose do sensor **medida** (2026-07-09), nao
arbitrada. O `lidar_yaw` e indispensavel: o zero do sensor nao aponta para a
frente do chassi, e sem ele o TF `base_link -> base_laser` sai girado 155 graus,
distorcendo o mapa do `slam_toolbox` e a odometria do `rf2o`.

| parametro   | valor      | como foi obtido                                  |
|-------------|------------|--------------------------------------------------|
| `lidar_x`   | 0.667 m    | 3 metodos independentes (espalhamento 1,3 cm)    |
| `lidar_y`   | 0.336 m    | trilateracao a partir dos centros das rodas      |
| `lidar_z`   | 0.15 m     | fita                                             |
| `lidar_yaw` | -2.70914 rad (-155.22 deg) | ajuste de reta em 2 paredes (+-0,25 deg) |

Os mesmos valores alimentam o `shared_control` (parametros `laser_x`, `laser_y`,
`laser_yaw_deg`), que os usa para o teste de corredor. Se o LiDAR for remontado,
**recalibre os dois lugares**.
