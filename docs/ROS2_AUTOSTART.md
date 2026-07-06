# Stack ROS 2 no boot (headless, sem GUI)

Objetivo: ao ligar a Raspberry Pi, o supervisor semi-assistido (e, opcionalmente,
a navegação) sobe automaticamente, **independente da interface gráfica**. A GUI
(`scripts/wheelchair_control_gui.py`) passa a ser só ferramenta de bancada/diagnóstico.

## Arquitetura

Uma **única ponte serial** (`esp_bridge`, do pacote `wheelchair_ros`) é a dona da
porta do ESP — ela casa com o firmware JSON atual e envia `drive_cfg`/`drive_cmd`.
A antiga `wheelchair_base/bridge_node` **não entra** neste fluxo (espera telemetria
com encoders que o firmware não emite e disputaria a mesma porta).

```
sllidar ──/scan──┐
                 ├─► shared_control ──/cmd_vel──► esp_bridge ──drive_cmd──► ESP
ESP ─telemetria──► esp_bridge ──/joystick_cmd_vel──┘
witmotion ─/imu_data─┐
rf2o ─/odom_rf2o─────┼─► EKF ─► slam_toolbox      (camada "nav", opcional)
```

A camada **nav** (`rf2o` + `EKF` + `slam_toolbox`) é pesada e só sobe com
`nav:=true`. O supervisor de segurança **não depende dela**.

## Estado validado (bancada, 2026-07)

Com `nav:=false armed:=false` (supervisor leve, sem motor), na Pi 5:

- `/scan` a **10,05 Hz** estável (LIDAR via `/dev/wheelchair/lidar`);
- ESP publicando `/joystick_cmd_vel` (via `/dev/wheelchair/esp32`);
- `shared_control` publicando `/cmd_vel` + `/wheelchair/assist_status`;
- **~4,6 GiB de RAM livres** (mesmo com o VS Code aberto) — o pipeline leve não
  pressiona. O OOM anterior era a camada **nav (slam+rf2o) junto com o IDE**.

Ainda **pendente de validação com motor** (rodas suspensas): `armed:=true` e a
parada física ao empurrar o joystick contra um obstáculo; e a camada `nav:=true`
medindo CPU/RAM.

## Instalação (uma vez)

```bash
cd ~/wheelchair-fw

# (1) compile/atualize os workspaces
cd ~/ros2_ws && colcon build --packages-select wheelchair_ros --symlink-install
cd ~/dev_ws  && colcon build --symlink-install

# (2) instale udev + systemd (NÃO habilita o boot ainda)
cd ~/wheelchair-fw
sudo bash scripts/install_wheelchair_service.sh
```

### Conferir os symlinks de dispositivo

Com ESP + LIDAR (+ IMU) conectados:

```bash
ls -l /dev/wheelchair/      # esperado: esp32, lidar, imu
```

Se faltar algum, os seriais nas regras estão diferentes do seu hardware:

```bash
bash scripts/gen_udev_rules.sh     # imprime os atributos reais
# ajuste ros2/wheelchair_ros/udev/99-wheelchair.rules e reinstale
```

> ⚠️ ESP e LIDAR usam o mesmo chip `10c4:ea60`; são distinguidos pelo
> **número de série** (ESP `0001`, LIDAR `b491…`). O IMU é `19f5:5740`.

## Teste escalonado (rodas suspensas!)

Suba do mais leve ao mais pesado, medindo CPU/RAM em outro terminal com `htop`
e `free -h`. Isso confirma que cabe na Pi antes de habilitar o boot.

**Etapa A — supervisor só, desarmado** (não move motor):  ✅ validado
```bash
WHEELCHAIR_LAUNCH_ARGS="nav:=false armed:=false" bash scripts/wheelchair_ros_boot.sh
```
Confira: `/scan` ~10 Hz, `/joystick_cmd_vel` reage ao joystick,
`/wheelchair/assist_status` mostra a zona.

**Etapa B — supervisor armado** (move motor, teto baixo):
```bash
WHEELCHAIR_LAUNCH_ARGS="nav:=false armed:=true max_duty:=0.20 assist_gain:=0.0" \
  bash scripts/wheelchair_ros_boot.sh
```
Joystick à frente com obstáculo perto → deve parar. Depois suba o desvio com
`assist_gain:=0.8`.

**Etapa C — stack completo** (liga nav/SLAM; aqui é onde a Pi pode saturar):
```bash
WHEELCHAIR_LAUNCH_ARGS="nav:=true armed:=true max_duty:=0.20 assist_gain:=0.8" \
  bash scripts/wheelchair_ros_boot.sh
```
Observe a RAM e a carga. Se `slam_toolbox`/`rf2o` saturarem, veja "Evitar o
congelamento" abaixo.

## Habilitar o boot automático

Só depois de validar a Etapa desejada:

```bash
# ajuste o default do boot (ex.: armar e ligar nav):
sudoedit /etc/default/wheelchair-ros      # WHEELCHAIR_LAUNCH_ARGS=...

sudo systemctl enable wheelchair-ros
sudo systemctl start  wheelchair-ros
journalctl -u wheelchair-ros -f
```

Operação:
```bash
systemctl status wheelchair-ros
sudo systemctl restart wheelchair-ros     # após editar o .env
sudo systemctl stop    wheelchair-ros
sudo systemctl disable wheelchair-ros     # tira do boot
```

## Evitar o congelamento (OOM) — o que causava e como mitigar

O travamento anterior foi **falta de memória** (`systemd-oomd` matando processos),
agravado por rodar o stack pesado **junto com o desktop e o VS Code**. No boot
headless via systemd, sem IDE, há muito mais folga. Ainda assim:

- **Energia**: use **hub USB alimentado** para o LIDAR (e IMU). O motor do LIDAR
  puxa corrente e causa *brownout* no Pi 5, derrubando dispositivos USB.
- **Não deixe o desktop/VS Code abertos** durante operação real — o serviço roda
  sem precisar deles.
- **Tunar o SLAM** (`~/dev_ws/src/wheelchair_base/config/slam.yaml`):
  - aumente `map_update_interval` (ex.: 5.0) e `minimum_time_interval`;
  - aumente `resolution` (ex.: 0.08) para mapa menor;
  - se não precisa mapear continuamente, considere `mode: localization`.
- **rf2o**: baixe `freq` (ex.: 7.0) se a CPU apertar.
- Se mesmo assim não couber: **boot só com `nav:=false`** (supervisor leve) e suba
  a navegação sob demanda quando precisar mapear.

## Segurança

- O serviço parte **desarmado** por padrão (`armed:=false`) — nenhum movimento até
  você ativar.
- Em `armed:=true`, todos os *failsafes* do firmware valem: perda de `/scan`, do
  nó de controle ou do USB levam a parada/desarme (ver `docs/pc4.tex`).
- Comece sempre com `max_duty` baixo e `assist_gain:=0.0` (só freio).
