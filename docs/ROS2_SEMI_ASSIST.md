# Controle semiassistido por LIDAR

Esta camada roda na Raspberry Pi com ROS 2 e adiciona assistência reativa de
obstáculos ao firmware atual. O joystick continua sendo a intenção primária da
pessoa; o LIDAR só reduz velocidade, adiciona uma correção limitada de direção
ou bloqueia o avanço.

## Arquitetura

```text
ESP32 firmware --telemetria drive--> esp_bridge --/joystick_cmd_vel-->
shared_control --/cmd_vel--> esp_bridge --drive_cfg + drive_cmd--> ESP32
                                      ^
                                      |
                                  /scan do sllidar_ros2
```

O firmware mantém as camadas de segurança:

- `drive_cfg` arma/desarma, define `max_duty`, `accel` e `decel`, e precisa
  chegar fresco ao ESP.
- `drive_cmd` traz os comandos normalizados das rodas calculados pela Pi.
- Se `drive_cmd` ficar velho, o firmware para o movimento assistido.
- `stop` desarma e limpa qualquer comando assistido pendente.

Sem `drive_cmd`, o firmware continua funcionando no modo manual atual: joystick
fisico lido pelo ESP e mixagem diferencial local.

## Pacote ROS 2

O pacote fica em `ros2/wheelchair_ros` e é um pacote `ament_python`.

```bash
cd ~/ros2_ws/src
ln -s /caminho/para/wheelchair-fw/ros2/wheelchair_ros wheelchair_ros
cd ~/ros2_ws
colcon build --packages-select wheelchair_ros --symlink-install
source install/setup.bash
```

Dependências esperadas:

- ROS 2 Jazzy
- `sllidar_ros2`
- `pyserial`

## Execução

Primeiro contato recomendado, com rodas suspensas e sem desvio autonomo:

```bash
ros2 launch wheelchair_ros wheelchair_assist.launch.py \
    esp_port:=/dev/ttyUSB1 \
    lidar_port:=/dev/ttyUSB0 \
    armed:=true \
    max_duty:=0.30 \
    assist_gain:=0.0
```

Depois de validar parada e freio suave, habilite o desvio:

```bash
ros2 param set /shared_control assist_gain 0.8
```

O launch inicia:

- `sllidar_ros2` publicando `/scan`;
- `shared_control` assinando `/scan` e `/joystick_cmd_vel`, publicando
  `/cmd_vel`;
- `esp_bridge` assinando `/cmd_vel`, publicando `/joystick_cmd_vel` e enviando
  `drive_cfg`/`drive_cmd` ao ESP.

Por segurança, `armed` nasce como `false`. Passe `armed:=true` apenas com a
cadeira suspensa ou em ambiente de teste controlado.

## Função de custo

Quando o usuário pede avanço (`linear.x > 0`), o `shared_control` avalia um
leque de direções candidatas dentro de `max_deviation_deg`.

```text
custo(delta) =
    w_obstacle * termo_obstaculo(folga(delta))
  + w_deviation * |delta - centro_da_intencao| / max_deviation
```

Onde:

- `folga(delta)` é a menor distância medida em um cone centrado na direção
  candidata.
- `termo_obstaculo` é zero acima de `slow_distance`, cresce entre
  `slow_distance` e `stop_distance`, e vira `blocked_cost` abaixo de
  `stop_distance`.
- `w_deviation` penaliza fugir da direção que a pessoa pediu.

Decisão:

- frente livre: passa o joystick intacto;
- obstáculo na faixa de atenção: reduz velocidade e curva em direção à menor
  função custo;
- todas as candidatas bloqueadas: zera o avanço e mantém a rotação pedida pelo
  usuário;
- sem `/scan` recente: bloqueia avanço, mantendo giro/ré conforme parâmetros.

## Tópicos úteis

| Tópico | Tipo | Uso |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | leituras do RPLIDAR |
| `/joystick_cmd_vel` | `geometry_msgs/Twist` | intenção do joystick |
| `/cmd_vel` | `geometry_msgs/Twist` | comando assistido final |
| `/wheelchair/assist_status` | `std_msgs/String` | JSON de modo/custos/folgas |
| `/wheelchair/telemetry_json` | `std_msgs/String` | telemetria crua do ESP |
| `/wheelchair/armed` | `std_msgs/Bool` | estado armado reportado pelo ESP |
| `/wheelchair/driving` | `std_msgs/Bool` | ESP aplicando comando |

## Parâmetros principais

`shared_control`:

| Parâmetro | Padrão | Descrição |
| --- | --- | --- |
| `stop_distance` | `0.45` | abaixo disso, direção bloqueada |
| `slow_distance` | `1.10` | abaixo disso, começa freio/desvio |
| `cone_half_deg` | `15.0` | meia-abertura do cone de folga |
| `max_deviation_deg` | `45.0` | limite de desvio |
| `num_candidates` | `19` | direções avaliadas |
| `assist_gain` | `0.8` | quanto do desvio escolhido aplicar |
| `scan_timeout_s` | `0.40` | sem scan, bloqueia avanço |

`esp_bridge`:

| Parâmetro | Padrão | Descrição |
| --- | --- | --- |
| `port` | `/dev/ttyUSB1` | porta do ESP |
| `baud` | `115200` | baud do ESP |
| `armed` | `false` | gate de segurança da ponte |
| `max_duty` | `0.30` | limite final aplicado pelo firmware |
| `accel` | `1.5` | rampa de aceleração |
| `decel` | `3.0` | rampa de frenagem |

## Limitações

- O RPLIDAR no plano frontal não protege ré nem degraus/desníveis fora do plano
  do feixe.
- O controle é reativo; ele não substitui planejamento global.
- A relação comando normalizado -> movimento real ainda depende da calibração
  mecanica e dos motores.
