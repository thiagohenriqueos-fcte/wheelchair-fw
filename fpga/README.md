# Acelerador em FPGA do supervisor LiDAR

Projeto final de **PCR / Co-Projeto Hardware-Software** (UnB, 2026/1).

Acelera em hardware o núcleo do supervisor de segurança da cadeira de rodas: a
projeção da varredura do LiDAR no `base_link` e o teste de corredor sobre 19
direções candidatas de desvio.

## Por que este bloco

É o nó mais pesado do sistema (17,7% de CPU na Pi 5; 8,3% após vetorizar com
NumPy) e o **único com latência crítica de segurança**: é ele que decide se a
cadeira freia. Custo por varredura:

| parte | custo | onde roda |
|---|---|---|
| projetar 720 feixes × testar 19 corredores | **O(N·K) = 9.918 avaliações** | **FPGA** |
| função custo + argmin | O(K) = 19 comparações | ARM (software) |

O `O(K)` fica em software de propósito: são 19 operações, e é ali que moram os
pesos que se ajustam em campo (`w_obstacle`, `w_deviation`, `assist_gain`).
Congelá-los em RTL trocaria flexibilidade por ganho irrisório.

## Resultados (medidos, não estimados)

Baseline na Raspberry Pi 5; ciclos de hardware medidos na simulação GHDL.

| implementação | tempo/varredura | speedup |
|---|---|---|
| software escalar (Python) | 3,54 ms | — |
| software vetorizado (NumPy) | 0,488 ms | 7× |
| **hardware @ 100 MHz** (751 ciclos) | **7,51 µs** | **472× / 65×** |
| hardware @ 150 MHz | 5,01 µs | 707× / 98× |

**Equivalência**: o testbench compara as 19 folgas produzidas pelo RTL com o
modelo de referência em ponto fixo, **bit a bit**, sobre três varreduras **reais**
capturadas na cadeira (com um balde no antigo ponto cego). As três passam.

## Precisão como parâmetro de projeto

O primeiro modelo de ponto fixo errava **402 mm**. A causa era real, não de
arredondamento: `θ = angle_min + i·inc + laser_yaw` chega a **−5,85 rad**, e o
formato Q3.13 comporta apenas ±4,0 rad — o ângulo **saturava** e o feixe era
projetado no lugar errado. Envolvendo θ em [−π, π):

| | erro médio | erro máximo |
|---|---|---|
| antes (θ saturado) | 17,89 mm | 402,15 mm |
| depois (θ envolvido) | **0,99 mm** | **5,27 mm** |

O LSB de Q6.10 é 0,98 mm: o erro final é de ~1 LSB, ou **0,9% da zona de parada**
de 600 mm.

## Arquitetura

```
(r, θ) → CORDIC → (cos, sin) → projeção → (X,Y) → 19 pistas em paralelo → 19 folgas
```

- **CORDIC** pipelinado, 16 estágios, **zero DSP** (só somadores e deslocamentos
  cabeados). Escolhido em vez de LUT de seno porque o ângulo depende de
  parâmetros de calibração reescritos em tempo de execução via AXI4-Lite.
- **19 pistas** com rotação de coeficiente **constante** → 4 multiplicadores fixos
  cada → ~78 DSP48 dos 220 do xc7z020 (35%). É aqui que o paralelismo paga: as
  19 pistas avaliam no mesmo ciclo o que o software itera.
- Vazão de **1 feixe/ciclo**, sem bolhas.

Interfaces: **AXI4-Stream** para os feixes (casa com AXI-DMA a partir da DDR),
**AXI4-Lite** para calibração e resultados, IRQ ao concluir.

## Como reproduzir

```bash
# simulação + prova de equivalência (roda na própria Pi)
bash sim/run_ghdl.sh

# síntese, timing, potência e recursos (no PC, com Vivado)
vivado -mode batch -source sim/build_vivado.tcl

# block design (ARM + AXI-DMA + acelerador) e a figura para o slide
vivado -mode batch -source sim/block_design.tcl
```

Relatórios em `vivado/reports/`: `timing_summary.rpt`, `utilization.rpt`,
`power.rpt`, `block_design.pdf`.

## Arquivos

| | |
|---|---|
| `golden/model.py` | referência float + modelo de ponto fixo **bit-exato**; gera estímulo e esperado |
| `golden/gen_pkg.py` | gera `rtl/cordic_pkg.vhd` a partir do modelo (constantes casam por construção) |
| `golden/capture_scans.py` | captura varreduras reais do `/scan` |
| `rtl/` | CORDIC, pista, núcleo, envelope AXI4 |
| `tb/tb_corridor_core.vhd` | testbench automático com leitura de arquivo |
| `sw/lidar_accel.c` | driver bare-metal do ARM (a parte O(K) do co-projeto) |
| `data/` | varreduras reais da cadeira |

**Aviso**: `rtl/cordic_pkg.vhd` é **gerado**. Não editar à mão — rode
`python3 golden/gen_pkg.py`.
