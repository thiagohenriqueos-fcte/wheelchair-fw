# Plano de teste: semiassistência por LIDAR

Valida a camada ROS 2 `wheelchair_ros` com o firmware que aceita
`drive_cfg`/`drive_cmd`.

## Segurança

As rodas devem ficar suspensas em todos os passos que podem acionar motor.
Comece com `max_duty:=0.30` ou menor. Nao teste com a cadeira no chao antes de
concluir este plano em bancada.

## Pré-requisitos

- ESP conectado, publicando telemetria `drive`.
- RPLIDAR C1 conectado e publicando `/scan`.
- ROS 2 Jazzy com `sllidar_ros2`.
- Pacote `wheelchair_ros` compilado:

```bash
cd ~/ros2_ws
colcon build --packages-select wheelchair_ros --symlink-install
source install/setup.bash
```

## A. Firmware e bridge

1. Compile o firmware.

   ```bash
   idf.py build
   ```

2. Rode somente o bridge, ainda desarmado.

   ```bash
   ros2 run wheelchair_ros esp_bridge --ros-args \
       -p port:=/dev/ttyUSB1 \
       -p armed:=false
   ```

3. Confirme telemetria crua.

   ```bash
   ros2 topic echo /wheelchair/telemetry_json --once
   ```

   Esperado: pacote `drive` com `x`, `y`, `out_left`, `out_right`.

4. Confirme intenção do joystick.

   ```bash
   ros2 topic echo /joystick_cmd_vel
   ```

   Joystick para frente deve gerar `linear.x` positivo.

## B. LIDAR isolado

1. Rode o RPLIDAR.

   ```bash
   ros2 launch sllidar_ros2 sllidar_c1_launch.py serial_port:=/dev/ttyUSB0
   ```

2. Confirme taxa.

   ```bash
   ros2 topic hz /scan
   ```

   Esperado: aproximadamente 10 Hz.

## C. Pipeline completo sem desvio

1. Suba tudo com `assist_gain:=0.0`.

   ```bash
   ros2 launch wheelchair_ros wheelchair_assist.launch.py \
       esp_port:=/dev/ttyUSB1 \
       lidar_port:=/dev/ttyUSB0 \
       armed:=true \
       max_duty:=0.20 \
       assist_gain:=0.0
   ```

2. Frente livre: empurre o joystick para frente.

   Esperado:

   - `/cmd_vel.linear.x` acompanha `/joystick_cmd_vel.linear.x`;
   - telemetria do ESP mostra `drive_mode:"assist"`;
   - rodas giram devagar e no sentido esperado.

3. Obstáculo dentro de `stop_distance` à frente.

   Esperado:

   - `/cmd_vel.linear.x` cai para zero;
   - `out_left` e `out_right` voltam a zero após a rampa/decel;
   - `/wheelchair/assist_status` mostra `mode:"para"`.

4. Obstáculo entre `stop_distance` e `slow_distance`.

   Esperado: `/cmd_vel.linear.x` reduz proporcionalmente à folga.

## D. Desvio

1. Habilite desvio.

   ```bash
   ros2 param set /shared_control assist_gain 0.8
   ```

2. Obstáculo à frente-direita, com espaço à esquerda.

   Esperado:

   - `/wheelchair/assist_status` mostra `mode:"desvia"`;
   - `/cmd_vel.angular.z` corrige para esquerda;
   - `drive_cmd` gera diferença entre rodas.

3. Obstáculo à frente-esquerda, com espaço à direita.

   Esperado: correção para direita.

4. Obstáculo largo cobrindo todas as candidatas.

   Esperado: `mode:"para"` e avanço zerado.

## E. Failsafes

1. Perda de `/scan`: encerre o LIDAR com o joystick pedindo frente.

   Esperado: após `scan_timeout_s`, `/cmd_vel.linear.x = 0` e
   `assist_status.mode:"sem_scan"`.

2. Perda do `shared_control`: encerre o nó.

   Esperado: após `cmd_timeout_s`, o bridge envia `stop`; no ESP,
   `drive_mode:"disarmed"` e motores zerados.

3. Perda do bridge/USB: encerre o bridge ou desconecte a serial.

   Esperado: após o timeout de `drive_cfg`, o firmware desarma e zera motores.

4. `drive_cmd` velho com `drive_cfg` ainda fresco.

   Simule interrompendo apenas `/cmd_vel`.

   Esperado: o firmware entra em `drive_mode:"assist_timeout"` e para, sem
   voltar sozinho para manual.
