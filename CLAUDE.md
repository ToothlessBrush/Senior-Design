# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Summary

Bare-metal quadcopter flight controller firmware for the STM32F411CEUx (Black Pill). Written in C with CMSIS, no HAL. Cross-compiled with `arm-none-eabi-gcc`.

- **MCU**: STM32F411CEUx @ 100 MHz (25 MHz HSE × 4 PLL)
- **Clocks**: APB1=50 MHz, APB2=100 MHz
- **Memory**: 128 KB RAM, 512 KB Flash

## Build Commands

Requires: `arm-none-eabi-gcc`, `cmake`, `make`, `openocd`, `st-flash`

```bash
mkdir build && cd build
cmake ..
make                           # Build firmware (.elf + .bin)
make flash                     # Flash to STM32 via st-flash
make debug                     # Start OpenOCD + GDB session
make unit_tests                # Build and run host-native unit tests
make flash_test_optical_flow   # Flash optical flow hardware test
```

Unit tests compile natively (not cross-compiled) and run on the host. They are in `tests/unit/`.

## Architecture

### Control Pipeline (6660 Hz, driven by IMU DRDY interrupt on PA0/EXTI0)

```
IMU raw data → bias calibration → biquad LPF (50 Hz gyro, 25 Hz accel)
  → complementary filter (attitude: roll/pitch/yaw in NED)
  → high-pass accel integration (velocity estimate)
  → outer PID loop (velocity → attitude setpoint correction)
  → inner PID loop (attitude → motor commands)
  → motor mixing (X-config) → DShot output
```

### Task Scheduler (`src/scheduler.c`)

Cooperative (non-preemptive). `main()` loops calling `scheduler_run()`. Tasks:
- `task_imu_pid`: uncapped (runs every iteration via IMU interrupt flag)
- `task_crsf_service`: uncapped (polls UART1 RX buffer continuously)
- `task_optical_flow`: 50 Hz
- `task_config_service`: 20 Hz (processes LoRa commands, sends telemetry)
- `task_led`: 2 Hz

### UART Driver (`src/uart.c`)

Three instances accessed by index enum:
- `UART_INSTANCE_1` (index 2): USART1, PA9/PA10, 500 kbaud — ELRS receiver. DMA2 Stream2 Ch4 (RX circular, 1024-byte buffer), DMA2 Stream7 Ch4 (TX). Idle-line ISR snaps `rx_head`.
- `UART_INSTANCE_2` (index 0): USART2, PA2/PA3, 115200 — LoRa. RXNEIE interrupt RX, DMA1 Stream6 TX.
- `UART_INSTANCE_6` (index 1): USART6, PA11/PA12, 115200 — Optical flow. RXNEIE interrupt RX, DMA2 Stream6 TX.

`uart_data_available()`, `uart_bytes_available()`, `uart_flush()`, and `uart_read_byte()` all call `uart_sync_dma_rx_head()` first to keep the circular buffer head current.

### Communication Protocols

- **CRSF** (`src/crsf.c`): ExpressLRS receiver protocol. 16 channels, 11-bit resolution, 988–2012 µs. Ch4 is arm switch (>1700 arm, <1300 disarm). 1000 ms timeout triggers failsafe.
- **LoRa Protocol** (`src/protocol.c`): Binary commands hex-encoded as `"PREFIX:HEXDATA"`. Commands: `HB` (heartbeat with throttle+attitude), `SP` (setpoint), `TP` (tune PID), `MB` (motor bias), `CF` (full config sync). Text commands: `FC:START`, `FC:STOP`, `FC:CALIBRATE`, `FC:RESET`, `FC:MANUAL`, `ES` (emergency stop).
- **Optical Flow** (`src/optical_flow.c`): MTF-01 sensor, Microlink protocol, checksum-validated frames.

### Motor Mixing (X-configuration)

```
Motor1 rear-left  (CW):  throttle - pitch + yaw
Motor2 front-right (CCW): throttle + roll - yaw  (wait — check motor_control.c for exact signs)
Motor3 rear-right  (CCW): throttle - roll - yaw
Motor4 front-left  (CW):  throttle + pitch + yaw
```

Throttle range: 47–1023 (DShot). Application throttle: 0.05–1.0.

### Key Hardware Notes

- USART1 BRR=200 (100 MHz / 500000 = 200 exactly, zero error)
- IDLE flag cleared on F4 by reading SR then DR (no ICR register; safe because DMA already consumed the byte)
- LED on PC13 is active-low
- IMU DRDY on PA0 → EXTI0 ISR sets a flag; `task_imu_pid` polls and clears it

## File Layout

```
src/           Main firmware sources
inc/           CMSIS device headers and sensor-specific headers
tests/unit/    Host-native unit tests (test_crsf.c, test_optical_flow_parse.c)
tests/         Flashable hardware tests
cmake/         Toolchain file (stm32f411.cmake)
linker/        STM32F411 linker script
vendor/        External dependencies
PINOUT.md      Complete pin mapping reference
```
