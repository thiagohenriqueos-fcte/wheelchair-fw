# ROS 2 Integration — Camada de Controle Semiassistido (host-side)

Esta camada roda na **Raspberry Pi** e adiciona assistência de desvio de
obstáculos por LIDAR sobre o firmware existente do ESP32-S3. O firmware **não
muda**: a Pi conversa com ele pelo mesmo protocolo JSON serial já validado nos
test plans v0.4–v0.7.

## Filosofia: semiassistido, não autônomo

O **joystick é o comando primário**. A pessoa dirige. A camada do LIDAR apenas
**modula** esse comando — nunca cria movimento que a pessoa não pediu. O
sistema só pode:

- **deixar passar** o comando do joystick intacto (frente livre);
- **reduzir** a velocidade e **corrigir** a direção dentro de uma janela
  limitada (desvio reativo);
- **parar** o avanço quando não há direção segura.

Invariante de segurança: a saída nunca adiciona velocidade linear para frente
além da que o usuário pediu, e nunca inverte o sentido de rotação pedido.

## Onde isto se encaixa no ROADMAP

Esta camada corresponde, no roadmap, à **v1.3 (ROS 2 serial bridge
integration)** e adiciona uma capacidade nova de **assistência de segurança por
LIDAR** que ainda não estava listada. A conversão velocidade→PWM por roda é
feita **open-loop na Pi** (papel previsto para a v0.9 no firmware); enquanto o
RPM real (v0.8) não estiver calibrado, as distâncias de parada são empíricas.

## Arquitetura

```
                          /joystick_cmd_vel
   [ ESP32-S3 ] --serial--> [ esp_bridge ] -----------------+
       ^  |                       |  ^                       |
       |  | telemetria joy/ack    |  | /cmd_vel              v
       |  +-----------------------+  |              [ shared_control ]
       |       pwm_test / stop       |                       ^
       +-----------------------------+                       | /scan
                                                    [ sllidar_ros2 (C1) ]
```

Fluxo por ciclo:

1. O ESP envia telemetria `joy` (inclui a leitura do joystick `x`, `y`).
2. `esp_bridge` decodifica a intenção do usuário e publica em
   `/joystick_cmd_vel`.
3. `shared_control` funde `/joystick_cmd_vel` + `/scan`, calcula o custo por
   direção e publica o comando assistido em `/cmd_vel`.
4. `esp_bridge` converte `/cmd_vel` em `pwm_test` por roda e envia ao ESP a
   20 Hz, alimentando o watchdog de 500 ms do firmware.

## Hardware

### LIDAR — RPLIDAR C1

| Item | Valor |
| --- | --- |
| Tecnologia | DTOF (Direct Time-of-Flight) |
| Alcance | 12 m (objeto claro); ~6 m (objeto escuro) |
| Zona cega | 0,05 m |
| Frequência | 10 Hz (600 rpm) |
| Resolução angular | 0,72° |
| Interface | UART 3,3 V via adaptador USB |
| **Baud** | **460800** |
| Classe laser | Class 1 (seguro aos olhos) |

Montagem: mantenha o sensor **recuado da borda dianteira** da cadeira para que
a zona cega de 5 cm não fique sobre a estrutura. Defina a frente do sensor
alinhada à frente da cadeira; se houver rotação, corrija via TF
`base_link` → `laser` (ver Calibração).

### ESP32-S3

Sem alteração. Pinagem em `docs/PINOUT.md`. Conexão USB à Pi em
`/dev/ttyACM0` (típico). O LIDAR usa outra porta, tipicamente `/dev/ttyUSB0`.

## Pré-requisitos de software

- Raspberry Pi 5, Ubuntu 24.04 (64 bits), ROS 2 **Jazzy**.
- Driver do LIDAR e `pyserial`:

```bash
sudo apt update
sudo apt install ros-jazzy-sllidar-ros2
python3 -m pip install pyserial        # ou apt: python3-serial
```

- Acesso às portas seriais (uma vez; relogar depois):

```bash
sudo usermod -aG dialout $USER
```

## Estrutura do pacote ROS 2

Crie um pacote Python `wheelchair_ros` no seu workspace colcon e coloque os três
arquivos:

```
~/ros2_ws/src/wheelchair_ros/
├── package.xml
├── setup.py
├── resource/wheelchair_ros
├── launch/
│   └── wheelchair_assist.launch.py
└── wheelchair_ros/
    ├── __init__.py
    ├── esp_bridge_node.py
    └── shared_control_node.py
```

`setup.py` — entry points:

```python
entry_points={
    "console_scripts": [
        "esp_bridge = wheelchair_ros.esp_bridge_node:main",
        "shared_control = wheelchair_ros.shared_control_node:main",
    ],
},
data_files=[
    ("share/wheelchair_ros/launch", ["launch/wheelchair_assist.launch.py"]),
    ("share/ament_index/resource_index/packages",
     ["resource/wheelchair_ros"]),
    ("share/wheelchair_ros", ["package.xml"]),
],
```

`package.xml` — dependências de execução:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>sllidar_ros2</exec_depend>
```

Build e ativação:

```bash
cd ~/ros2_ws
colcon build --packages-select wheelchair_ros --symlink-install
source install/setup.bash
```

## Nós

### `esp_bridge`

Ponte serial bidirecional entre o ESP32 e o grafo ROS 2.

**Assina**

| Tópico | Tipo | Uso |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | comando final → `pwm_test` por roda |

**Publica**

| Tópico | Tipo | Uso |
| --- | --- | --- |
| `/joystick_cmd_vel` | `geometry_msgs/Twist` | intenção decodificada do joystick |
| `/wheelchair/telemetry_json` | `std_msgs/String` | pacote JSON cru para inspeção |
| `/wheelchair/motor_active` | `std_msgs/Bool` | espelho de `motor_test_active` |
| `/wheelchair/enc_left_count` | `std_msgs/Int32` | contagem do encoder esquerdo |
| `/wheelchair/enc_right_count` | `std_msgs/Int32` | contagem do encoder direito |

**Parâmetros**

| Parâmetro | Padrão | Descrição |
| --- | --- | --- |
| `port` | `/dev/ttyACM0` | porta serial do ESP |
| `baud` | `115200` | baud do ESP |
| `cmd_rate_hz` | `20.0` | taxa de envio (mantém o watchdog vivo) |
| `cmd_timeout_s` | `0.3` | sem `/cmd_vel` → envia `stop` |
| `max_duty` | `0.30` | gate de segurança (idêntico ao GUI) |
| `gain_lin` | `1.0` | ganho linear → duty |
| `gain_ang` | `0.5` | ganho angular → duty |
| `joy_v_scale` | `1.0` | `y` → `linear.x` |
| `joy_w_scale` | `1.0` | `-x` → `angular.z` |

Conversão Twist → duty por roda (open-loop, **não calibrada**):

```
left  = clamp(gain_lin * v - gain_ang * w, -max_duty, +max_duty)
right = clamp(gain_lin * v + gain_ang * w, -max_duty, +max_duty)
```

Comandos emitidos ao ESP (protocolo inalterado):

```json
{"type":"pwm_test","seq":N,"left":L,"right":R}
{"type":"stop","seq":N}
```

Decodificação do joystick (telemetria → intenção), convenção
`y>0` = frente, `x>0` = direita, ROS REP-103 (`+angular.z` = esquerda):

```
linear.x  =  joy_v_scale * y
angular.z = -joy_w_scale * x
```

### `shared_control`

Controlador semiassistido por **menor custo**.

**Assina**: `/joystick_cmd_vel`, `/scan`
**Publica**: `/cmd_vel`

**Parâmetros**

| Parâmetro | Padrão | Descrição |
| --- | --- | --- |
| `stop_distance` | `0.45` m | abaixo disto, para o avanço |
| `slow_distance` | `1.10` m | abaixo disto, começa a frear/desviar |
| `cone_half_deg` | `15.0` | meia-largura do cone p/ medir folga |
| `max_deviation_deg` | `45.0` | Δmax: limite do desvio |
| `num_candidates` | `19` | nº de direções avaliadas |
| `w_obstacle` | `1.0` | peso do termo de obstáculo |
| `w_deviation` | `0.35` | peso do termo de desvio |
| `blocked_cost` | `10.0` | custo de candidata bloqueada |
| `assist_gain` | `0.8` | fração da correção de curva aplicada |
| `allow_reverse` | `true` | ré passa direto (sem visão traseira) |
| `reverse_speed_cap` | `0.5` | fração de `v` na ré |
| `scan_timeout_s` | `0.4` | sem `/scan` → bloqueia avanço |
| `control_rate_hz` | `20.0` | taxa do loop de controle |

**Função de custo.** A cada ciclo, com o usuário pedindo frente (`v > 0`),
avalia-se um leque de direções candidatas `δ` em torno da curva pedida:

```
custo(δ) = w_obstacle * termo_obstaculo(folga(δ)) + w_deviation * |δ| / Δmax

folga(δ)        = menor distância medida num cone de meia-largura cone_half
                  centrado em δ
termo_obstaculo = 0                         se folga >= slow_distance
                  (slow-folga)/(slow-stop)  se stop < folga < slow   (0..1)
                  blocked_cost              se folga <= stop_distance
```

Seleciona `δ* = argmin custo(δ)`. Decisão:

- **todas bloqueadas** ou folga à frente ≤ `stop_distance` → **para** o avanço
  (`linear.x = 0`), mantém a rotação do usuário para ele escapar;
- caso contrário → **segue**: `linear.x` reduzido proporcional à folga à frente
  (freio suave entre `slow` e `stop`) e `angular.z = w_user + assist_gain * δ*`;
- folga à frente ≥ `slow_distance` → **passa intacto** (intervenção zero).

**Ré**: o LIDAR frontal não vê atrás; `v < 0` passa direto (com `reverse_speed_cap`).

## Tópicos — resumo

| Tópico | Tipo | Produtor → Consumidor |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | sllidar_ros2 → shared_control |
| `/joystick_cmd_vel` | `geometry_msgs/Twist` | esp_bridge → shared_control |
| `/cmd_vel` | `geometry_msgs/Twist` | shared_control → esp_bridge |
| `/wheelchair/*` | vários | esp_bridge → diagnóstico |

## Execução

```bash
ros2 launch wheelchair_ros wheelchair_assist.launch.py \
    esp_port:=/dev/ttyACM0 lidar_port:=/dev/ttyUSB0 lidar_baud:=460800
```

Primeiro contato recomendado: subir com `assist_gain:=0.0` (só **freia e
para**, sem curvar) e rodas suspensas. Depois de confiar, suba o ganho para
habilitar o desvio.

## Modelo de segurança (camadas independentes)

1. **Watchdog do firmware (500 ms)** — se a Pi parar de enviar, o ESP zera os
   motores sozinho.
2. **Timeout do bridge** — sem `/cmd_vel` em `cmd_timeout_s`, o bridge envia
   `stop`.
3. **Timeout de sensor** — sem `/scan` em `scan_timeout_s`, o `shared_control`
   bloqueia o avanço.
4. **Stop ao encerrar** — Ctrl+C em qualquer nó emite `stop`/Twist zero antes
   de sair.
5. **Gate de duty** — `max_duty = 0.30` no bridge, igual ao gate do GUI.
6. **Invariante semiassistido** — nunca adiciona avanço além do pedido.

Procedimento físico: **sempre teste com as rodas suspensas** (igual ao
`TEST_PLAN_V0_6.md`) antes de qualquer teste com a cadeira no chão.

## Calibração

- **Ângulo zero / TF**: defina a transformada estática `base_link` → `laser`
  conforme a montagem real. Se o sensor estiver girado, isto evita ter de
  corrigir offset no código. Verifique no RViz qual direção do `/scan`
  corresponde à frente da cadeira.
- **Sinais do joystick**: ajuste `joy_v_scale`/`joy_w_scale` (sinal incluso) se
  frente/lado vierem invertidos.
- **Distâncias `stop`/`slow`**: meça a distância de frenagem real da cadeira
  **com carga** e ajuste com margem. Os padrões são conservadores e empíricos.
- **Velocidade open-loop**: a relação comando→velocidade real só fica precisa
  após a calibração de RPM (v0.8) e o mapa v/w→PWM (v0.9).

## Limitações conhecidas

- **Sem visão traseira**: a ré não é protegida (LIDAR frontal). Para proteção
  em marcha à ré, adicione um sensor traseiro.
- **Plano 2D único**: obstáculos muito baixos, degraus/desníveis e objetos
  acima do plano do feixe não são vistos.
- **Velocidade não calibrada**: open-loop até v0.8/v0.9.
- **Reativo, não planejado**: é assistência de desvio, não navegação. Para
  planejamento global use Nav2 numa etapa futura.

## Arquivos

| Arquivo | Local sugerido |
| --- | --- |
| `esp_bridge_node.py` | `wheelchair_ros/wheelchair_ros/` |
| `shared_control_node.py` | `wheelchair_ros/wheelchair_ros/` |
| `wheelchair_assist.launch.py` | `wheelchair_ros/launch/` |

## Referências

- Driver: `Slamtec/sllidar_ros2` (launch `sllidar_c1_launch.py`, baud 460800).
- Convenção de eixos: REP-103 (`+x` frente, `+z` angular = esquerda).
- Protocolo serial do ESP: ver `TEST_PLAN_V0_4.md` e `TEST_PLAN_V0_6.md`.
