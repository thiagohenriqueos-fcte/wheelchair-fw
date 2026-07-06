#!/usr/bin/env bash
# Instala as regras udev e o serviço systemd do stack ROS 2 da cadeira.
#
#     sudo bash scripts/install_wheelchair_service.sh
#
# NÃO habilita nem inicia o serviço — só instala os arquivos. Isso é proposital:
# habilite o boot só DEPOIS de validar em bancada (rodas suspensas). No fim, o
# script imprime os comandos para habilitar.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "Rode com sudo:  sudo bash scripts/install_wheelchair_service.sh" >&2
    exit 1
fi

REPO="$(cd "$(dirname "$0")/.." && pwd)"
echo "Repositório: $REPO"

# 1) Regras udev (symlinks /dev/wheelchair/*)
install -m 0644 "$REPO/ros2/wheelchair_ros/udev/99-wheelchair.rules" \
        /etc/udev/rules.d/99-wheelchair.rules
echo "[ok] /etc/udev/rules.d/99-wheelchair.rules"
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty
echo "[ok] udev recarregado"

# 2) Serviço systemd
install -m 0644 "$REPO/systemd/wheelchair-ros.service" \
        /etc/systemd/system/wheelchair-ros.service
echo "[ok] /etc/systemd/system/wheelchair-ros.service"

# 3) Arquivo de configuração (preserva edições existentes)
if [ ! -f /etc/default/wheelchair-ros ]; then
    install -m 0644 "$REPO/systemd/wheelchair-ros.env" /etc/default/wheelchair-ros
    echo "[ok] /etc/default/wheelchair-ros (default seguro: nav=true, armed=false)"
else
    echo "[skip] /etc/default/wheelchair-ros já existe (mantido)"
fi

# 4) Garante o bit de execução no lançador
chmod +x "$REPO/scripts/wheelchair_ros_boot.sh"
systemctl daemon-reload
echo "[ok] systemd daemon-reload"

cat <<EOF

Instalado. Próximos passos (NÃO habilitei o boot por segurança):

  1. Conecte ESP + LIDAR (+ IMU) e confira os symlinks:
       ls -l /dev/wheelchair/
     (Se faltar algum, rode  bash scripts/gen_udev_rules.sh  e ajuste os
      seriais em ros2/wheelchair_ros/udev/99-wheelchair.rules, reinstale.)

  2. Teste manual, rodas suspensas, começando leve:
       systemctl start wheelchair-ros      # usa /etc/default/wheelchair-ros
       journalctl -u wheelchair-ros -f
     ou direto:
       WHEELCHAIR_LAUNCH_ARGS="nav:=false armed:=false" \\
         bash scripts/wheelchair_ros_boot.sh

  3. Validado em bancada? Habilite o boot automático:
       sudo systemctl enable wheelchair-ros

  Para parar/desabilitar:
       sudo systemctl stop wheelchair-ros
       sudo systemctl disable wheelchair-ros
EOF
