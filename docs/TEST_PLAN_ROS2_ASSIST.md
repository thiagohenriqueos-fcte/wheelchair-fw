# Test Plan: ROS 2 Assisted Control (LIDAR semiassistido)

Esta camada host-side adiciona assistência de desvio de obstáculos por LIDAR
sobre o firmware do ESP32-S3. **O firmware não muda** — o protocolo serial é o
mesmo dos test plans v0.4–v0.7. Corresponde à v1.3 do roadmap (ponte ROS 2)
mais a nova assistência de segurança por LIDAR.

Componentes validados:
- `sllidar_ros2` publicando `/scan` (RPLIDAR C1).
- `esp_bridge` — ponte serial ↔ ROS 2.
- `shared_control` — controle compartilhado por menor custo.

## DANGER — Motor suspenso

**As rodas devem estar suspensas, livres e sem carga em todos os passos que
acionam o motor.** Um erro de sinal ou de configuração pode causar movimento
inesperado. Prenda a cadeira antes de aplicar energia ao IBT-2. Não toque no
eixo do motor durante os testes.

## IMPORTANTE — Semiassistido, não autônomo

O joystick é o comando primário; o LIDAR só modula. Este plano valida que a
camada **reduz, corrige ou para** o comando do usuário — nunca que ela dirige
sozinha. Comece sempre pelo modo "só parar" (`assist_gain:=0.0`).

## Pré-requisitos

- ESP32-S3 com firmware **v0.6.0 ou posterior** (watchdog de 500 ms ativo),
  conectado em `/dev/ttyACM0`.
- RPLIDAR C1 conectado, tipicamente `/dev/ttyUSB0`, baud 460800.
- Joystick ligado conforme `docs/PINOUT.md` (3,3 V).
- Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy.
- Pacote `wheelchair_ros` compilado e o ambiente ativado:
  `source ~/ros2_ws/install/setup.bash`.
- Usuário no grupo `dialout`.
- Nenhum outro processo segurando `/dev/ttyACM0` (feche `idf.py monitor`,
  GUIs e scripts seriais).

## Build do pacote

```bash
cd ~/ros2_ws
colcon build --packages-select wheelchair_ros --symlink-install
source install/setup.bash
```

Confirme que `ros2 pkg executables wheelchair_ros` lista `esp_bridge` e
`shared_control`.

## A. LIDAR isolado (sem o resto)

- [ ] 1. Suba o C1 com visualização:

  ```bash
  ros2 launch sllidar_ros2 view_sllidar_c1_launch.py serial_port:=/dev/ttyUSB0
  ```

- [ ] 2. Confirme no log `SLLidar health status : OK`.
- [ ] 3. Confirme no RViz a nuvem de pontos girando a ~10 Hz.
- [ ] 4. Identifique qual direção do `/scan` corresponde à **frente da
      cadeira**. Anote para a calibração do TF / sinais.
- [ ] 5. Em outro terminal, confirme a taxa:

  ```bash
  ros2 topic hz /scan
  ```

  Esperado ~10 Hz. Encerre o launch antes de seguir.

## B. Ponte serial (`esp_bridge`)

- [ ] 6. Rode só a ponte:

  ```bash
  ros2 run wheelchair_ros esp_bridge --ros-args -p port:=/dev/ttyACM0
  ```

- [ ] 7. Confirme que o nó abre a porta sem erro (log "esp_bridge pronto").
- [ ] 8. Verifique a telemetria crua:

  ```bash
  ros2 topic echo /wheelchair/telemetry_json --once
  ```

  Deve mostrar um pacote JSON `joy` do firmware.

- [ ] 9. Verifique a intenção do joystick decodificada:

  ```bash
  ros2 topic echo /joystick_cmd_vel
  ```

  Mova o joystick para frente → `linear.x` positivo; para a direita →
  `angular.z` negativo. Centralizado → próximo de zero.

- [ ] 10. Confirme `/wheelchair/motor_active` em `false` (sem comando).
- [ ] 11. Confirme `/wheelchair/enc_left_count` e `enc_right_count` publicando.

## C. Caminho de comando (com rodas suspensas)

Publique `/cmd_vel` manualmente para validar a tradução para `pwm_test`.

- [ ] 12. Com `esp_bridge` rodando e um leitor à parte:

  ```bash
  python3 scripts/read_json_serial.py /dev/ttyACM0   # em outro terminal? ver nota
  ```

  > Nota: só um processo abre a porta do ESP. Para inspecionar o que o bridge
  > envia, use `ros2 topic echo /wheelchair/telemetry_json` (o próprio bridge já
  > republica a telemetria), em vez de abrir a porta de novo.

- [ ] 13. Comande frente baixa por 2 s:

  ```bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist \
      "{linear: {x: 0.10}, angular: {z: 0.0}}"
  ```

  Confirme em `/wheelchair/telemetry_json`:
  - `motor_test_active: true`,
  - `motor_left ≈ 0.10`, `motor_right ≈ 0.10` (clamp ≤ `max_duty`).

- [ ] 14. Pare o `ros2 topic pub` (Ctrl+C). Confirme que, após `cmd_timeout_s`,
      o bridge envia `stop` e `motor_test_active` vira `false`.

- [ ] 15. Confirme o gate de duty: comande `linear.x: 0.90`. Confirme que o
      motor não passa de `max_duty` (0,30) — `motor_left ≈ 0.30`.

## D. Controle compartilhado — modo "só parar" (`assist_gain = 0`)

- [ ] 16. Suba o pipeline completo, sem desvio:

  ```bash
  ros2 launch wheelchair_ros wheelchair_assist.launch.py \
      lidar_baud:=460800
  # depois, ajuste o ganho a zero para o primeiro teste:
  ros2 param set /shared_control assist_gain 0.0
  ```

- [ ] 17. Com a frente livre, empurre o joystick para frente. Confirme que
      `/cmd_vel` repassa `linear.x` (motor ativo, rodas suspensas girando).

- [ ] 18. Aproxime um obstáculo (a mão) da frente do LIDAR, dentro de
      `stop_distance`. Confirme:
      - `/cmd_vel` `linear.x` cai a 0,
      - `motor_test_active` vira `false`,
      - a rotação do joystick (se houver) continua passando.

- [ ] 19. Afaste o obstáculo para a faixa `stop`–`slow`. Confirme o **freio
      suave**: `linear.x` sai de 0 e cresce conforme a folga aumenta.

- [ ] 20. Afaste além de `slow_distance`. Confirme passagem intacta do joystick.

## E. Controle compartilhado — desvio (`assist_gain > 0`)

- [ ] 21. Habilite o desvio:

  ```bash
  ros2 param set /shared_control assist_gain 0.8
  ```

- [ ] 22. Coloque um obstáculo à frente-direita (dentro de `slow`, fora de
      `stop`), joystick para frente. Confirme que `/cmd_vel` ganha `angular.z`
      **para a esquerda** (afastando do obstáculo), com `linear.x` reduzido.

- [ ] 23. Espelhe à frente-esquerda. Confirme correção para a direita.

- [ ] 24. Bloqueie toda a frente (obstáculo largo dentro de `stop`). Confirme
      que volta ao comportamento de **parar** o avanço (nenhuma direção segura).

## F. Failsafes

- [ ] 25. **Perda de sensor**: com o joystick para frente e frente livre,
      mate o nó do LIDAR. Após `scan_timeout_s`, confirme que `/cmd_vel`
      bloqueia o avanço (`linear.x = 0`).

- [ ] 26. **Perda da ponte**: com motor ativo, mate `esp_bridge`. Confirme que,
      após ~500 ms, o **watchdog do firmware** zera os motores
      (`motor_test_active: false`).

- [ ] 27. **Stop ao encerrar**: com motor ativo, encerre o `shared_control`
      (Ctrl+C). Confirme `/cmd_vel` zero e motor parando.

- [ ] 28. **Ré sem visão traseira**: puxe o joystick para trás com um obstáculo
      só atrás (fora do campo do LIDAR). Confirme que a ré passa (com
      `reverse_speed_cap`) — comportamento esperado e documentado como
      limitação.

## Saídas esperadas (exemplos)

Intenção do joystick:

```yaml
# /joystick_cmd_vel  (joystick à frente)
linear:  {x: 0.62, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}
```

Comando assistido durante desvio:

```yaml
# /cmd_vel  (obstáculo à direita, frente parcialmente livre)
linear:  {x: 0.18, y: 0.0, z: 0.0}     # reduzido pelo freio suave
angular: {x: 0.0, y: 0.0, z: 0.34}     # corrige para a esquerda
```

Telemetria do ESP (republicada pelo bridge):

```json
{"type":"joy","seq":42,"fw":"0.7.0","x":0.30,"y":0.62,"motor_left":0.18,"motor_right":0.30,"motor_test_active":true,"enc_left_count":0,"enc_right_count":0,"enc_status":"ok","status":"ok"}
```

## Commit após validação

```bash
git add docs/ROS2_INTEGRATION.md docs/TEST_PLAN_ROS2_ASSIST.md \
        ros2/wheelchair_ros/
git commit -m "ros2: add LIDAR semi-assist layer (esp_bridge + shared_control)"
```

## Notes

- O firmware e o protocolo serial não mudam nesta camada.
- `assist_gain = 0.0` reduz tudo a "só freia e para" — use no primeiro contato.
- As distâncias `stop`/`slow` são empíricas até a calibração de RPM (v0.8).
- A ré não é protegida (LIDAR frontal). Documentado em `ROS2_INTEGRATION.md`.
- Não valide passos D–F com a cadeira no chão antes de concluir D–F com rodas
  suspensas.
