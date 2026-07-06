#!/usr/bin/env bash
# Gera/verifica as regras udev a partir dos dispositivos REALMENTE conectados.
#
# Rode com o ESP, o LIDAR e o IMU plugados:
#     bash scripts/gen_udev_rules.sh
#
# Ele imprime os atributos de cada /dev/ttyUSB*/ttyACM* e uma sugestão de regra.
# Compare com ros2/wheelchair_ros/udev/99-wheelchair.rules e ajuste os seriais
# se necessário. Não escreve em /etc — é só diagnóstico.

set -euo pipefail

echo "== Dispositivos seriais e seus atributos udev =="
shopt -s nullglob
found=0
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    found=1
    vid=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_VENDOR_ID=//p')
    pid=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_MODEL_ID=//p')
    ser=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_SERIAL_SHORT=//p')
    mdl=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_MODEL=//p')
    lnk=$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^DEVLINKS=//p')
    echo "--- $dev  (${mdl:-?}) ---"
    echo "    idVendor=$vid  idProduct=$pid  serial=$ser"
    echo "    SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\", ATTRS{serial}==\"$ser\", SYMLINK+=\"wheelchair/<nome>\", MODE=\"0666\""
    [ -n "$lnk" ] && echo "    links atuais: $lnk"
done
[ "$found" = 1 ] || echo "(nenhum /dev/ttyUSB*/ttyACM* encontrado — conecte os dispositivos)"

echo
echo "== Symlinks /dev/wheelchair atuais =="
ls -l /dev/wheelchair/ 2>/dev/null || echo "(ainda não existem)"
