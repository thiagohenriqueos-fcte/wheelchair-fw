# Rodar o Vivado no seu PC

O Vivado não existe na Raspberry Pi (é x86). Este guia diz **o que copiar** e
**como rodar**.

## 1. O que copiar

Copie a pasta **`fpga/`** inteira. São ~200 KB — cabe num pendrive, num
`scp`, num zip por e-mail. Não precisa de mais nada do repositório.

Dentro dela, o que o Vivado realmente usa:

```
fpga/
├── rtl/                  <-- IMPRESCINDÍVEL: o hardware
│   ├── cordic_pkg.vhd        (gerado; NÃO editar à mão)
│   ├── cordic.vhd
│   ├── lane.vhd
│   ├── corridor_core.vhd
│   └── lidar_accel_axi.vhd   (o topo, com AXI4)
├── tb/                   <-- para simular dentro do Vivado (opcional)
│   ├── tb_corridor_core.vhd
│   ├── stimulus_*.txt        (as 3 varreduras reais)
│   └── expected_*.txt
└── sim/
    ├── build_vivado.tcl      <-- síntese + timing + potência + recursos
    └── block_design.tcl      <-- ARM + AXI-DMA + acelerador (a figura)
```

O resto (`golden/`, `data/`, `sw/`, `docs/`) não é necessário para o Vivado,
mas leve junto — é o que sustenta o relatório.

### Um comando para empacotar

Na Pi:
```bash
cd ~/wheelchair-fw
tar czf fpga.tar.gz fpga/
```
Depois copie `fpga.tar.gz` para o PC (pendrive, `scp`, o que for) e extraia.

## 2. ANTES de rodar: escolha a placa

Abra **os dois** `.tcl` e confira a linha do `PART`:

| placa | linha |
|---|---|
| **ZedBoard** (padrão) | `set PART "xc7z020clg484-1"` |
| **PYNQ-Z1** | `set PART "xc7z020clg400-1"` |

É o **mesmo chip** (xc7z020) — muda só o encapsulamento. O VHDL é idêntico.

## 3. Rodar

Abra o **Vivado Tcl Shell** (ou o prompt com `vivado` no PATH) e:

```bash
cd caminho/para/fpga          # entre NA pasta fpga (os scripts usam caminho relativo)

# (a) síntese + implementação + timing + potência + recursos
vivado -mode batch -source sim/build_vivado.tcl

# (b) block design (ARM + AXI-DMA + acelerador) + a figura para o slide
vivado -mode batch -source sim/block_design.tcl
```

Cada um leva alguns minutos. O `build_vivado.tcl` **imprime um resumo no
console** ao final, já com os números da tabela do relatório:

```
================ RESUMO PARA O RELATORIO ================
  Periodo alvo    : 10.00 ns (100 MHz)
  WNS (folga)     : +X.XXX ns
  Fmax            : XXX.X MHz
  Potencia total  : X.XXX W
  LUTs            : XXXX
  Flip-flops      : XXXX
  DSP48           : XX
  Latencia (sim)  : 751 ciclos = 7,51 us @ 100 MHz
=========================================================
```

## 4. O que sai, e para onde vai

Tudo em **`fpga/vivado/reports/`**:

| arquivo | onde usar |
|---|---|
| `timing_summary.rpt` | WNS e Fmax → Tabela V do relatório |
| `timing_paths.rpt` | os 10 piores **caminhos críticos** (print para o slide) |
| `utilization.rpt` | LUT / FF / DSP / BRAM → Tabela V |
| `utilization_hier.rpt` | consumo **por módulo** (mostra o CORDIC com 0 DSP) |
| `power.rpt` | potência total e por bloco |
| `block_design.pdf` / `.png` | **a figura do block design** para o slide |

No relatório e nos slides, os campos pendentes estão marcados em **vermelho**
(`\vivado{---}` / `[VIVADO]`) — impossível esquecer algum.

## 5. Se algo falhar

- **`ERROR: [Common 17-70] Application Exception`** ao empacotar o IP no
  `block_design.tcl`: rode primeiro o `build_vivado.tcl` (ele valida o RTL
  sozinho, sem block design). Se o RTL sintetizar, o problema é do empacotamento
  — dá para montar o block design pela GUI, importando o IP de `vivado/ip_repo`.
- **Versão do Vivado**: os scripts usam APIs estáveis desde 2019.2. Se sua
  versão reclamar de `write_bd_layout -format pdf`, troque para `-format png`.
- **Simular dentro do Vivado** (opcional — o GHDL já provou a equivalência):
  o `build_vivado.tcl` já adiciona o testbench ao `sim_1`. Rode
  `launch_simulation` na GUI. Os `stimulus_*.txt` precisam estar acessíveis:
  ajuste o generic `TB_DIR` para o caminho absoluto da pasta `tb/`.

## 6. Reproduzir a prova de equivalência (não precisa de Vivado)

Na Pi, ou em qualquer máquina com GHDL:
```bash
bash sim/run_ghdl.sh
```
Gera o estímulo, compila, simula as 3 varreduras reais e compara bit a bit.
