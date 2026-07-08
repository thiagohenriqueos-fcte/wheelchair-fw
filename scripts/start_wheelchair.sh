#!/usr/bin/env bash
# Sobe TODOS os nós da cadeira via wheelchair_full.launch.py.
#
# Rode no SEU terminal (fica no primeiro plano; Ctrl-C encerra tudo):
#
#   bash ~/wheelchair-fw/scripts/start_wheelchair.sh            # DESARMADO (padrão seguro)
#   bash ~/wheelchair-fw/scripts/start_wheelchair.sh armed      # armado p/ teste (max_duty 0.20)
#   bash ~/wheelchair-fw/scripts/start_wheelchair.sh drive      # armado + navegação + desvio
#
# Argumentos livres via ARGS (sobrescreve tudo):
#   ARGS="nav:=true armed:=true max_duty:=0.40 assist_gain:=0.8" \
#     bash ~/wheelchair-fw/scripts/start_wheelchair.sh
#
# Camadas (ver o launch): sempre sobe LIDAR + esp_bridge + shared_control (+ IMU).
# nav:=true adiciona rf2o + EKF + slam_toolbox (odometria/pose; mais pesado).
#
# OBS: NÃO usar 'set -u' — os setup.bash do ROS referenciam variáveis não
# definidas e quebrariam o source no systemd/terminal.

source /opt/ros/jazzy/setup.bash
source /home/wheelchair/dev_ws/install/setup.bash
source /home/wheelchair/ros2_ws/install/setup.bash

# Presets por atalho no 1o argumento; senão, default desarmado e seguro.
case "${1:-}" in
  armed)
    ARGS="nav:=false imu:=false armed:=true max_duty:=0.20 assist_gain:=0.0" ;;
  drive)
    ARGS="nav:=true armed:=true max_duty:=0.40 assist_gain:=0.8" ;;
  *)
    ARGS="${ARGS:-nav:=false imu:=false armed:=false max_duty:=0.20 assist_gain:=0.0}" ;;
esac

echo ">> ros2 launch wheelchair_ros wheelchair_full.launch.py ${ARGS}"
echo ">> (Ctrl-C encerra tudo com segurança: a ponte manda stop e o firmware desarma)"
exec ros2 launch wheelchair_ros wheelchair_full.launch.py ${ARGS}
