#!/usr/bin/env bash
# Lançador do stack ROS 2 da cadeira para uso headless / no boot (systemd).
#
# Faz o source do ROS e dos dois workspaces e sobe o launch unificado. Os
# argumentos do launch vêm de WHEELCHAIR_LAUNCH_ARGS (ver systemd/wheelchair-ros.env);
# o default é conservador: navegação ligada, porém DESARMADO (não move o motor).
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /home/wheelchair/dev_ws/install/setup.bash
source /home/wheelchair/ros2_ws/install/setup.bash

# Argumentos do launch (sobrescreva via env). Sem aspas: cada token é um arg.
ARGS=${WHEELCHAIR_LAUNCH_ARGS:-"nav:=true armed:=false"}

exec ros2 launch wheelchair_ros wheelchair_full.launch.py ${ARGS}
