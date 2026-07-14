#!/usr/bin/env bash
# Simulacao completa com GHDL: compila, elabora e roda o testbench contra as
# tres varreduras REAIS do LiDAR da cadeira, comparando bit a bit com o modelo
# de referencia em ponto fixo.
#
#   bash sim/run_ghdl.sh
set -e
cd "$(dirname "$0")/.."
mkdir -p sim/work

echo "== gerando estimulo a partir do golden model =="
python3 golden/gen_pkg.py
python3 golden/model.py stimulus

echo "== compilando =="
ghdl -a --std=08 --workdir=sim/work \
  rtl/cordic_pkg.vhd rtl/cordic.vhd rtl/lane.vhd rtl/corridor_core.vhd \
  tb/tb_corridor_core.vhd
ghdl -e --std=08 --workdir=sim/work -o sim/tb_corridor_core tb_corridor_core

echo "== simulando =="
fail=0
for s in 0 1 2; do
  echo "-- varredura $s --"
  if ./sim/tb_corridor_core -gSCAN_ID=$s --ieee-asserts=disable 2>&1 \
       | grep -qE "EQUIVALENCIA OK"; then
    echo "   OK: 19 folgas casam bit a bit com o modelo"
  else
    echo "   FALHOU"; fail=1
  fi
done

echo
if [ $fail -eq 0 ]; then
  echo "TODAS AS VARREDURAS PASSARAM -- RTL equivalente ao modelo de referencia."
else
  echo "HOUVE FALHA."; exit 1
fi

# onda para inspecao no GTKWave (opcional)
echo
echo "Para gerar a forma de onda:"
echo "  ./sim/tb_corridor_core -gSCAN_ID=0 --wave=sim/wave.ghw"
echo "  gtkwave sim/wave.ghw"
