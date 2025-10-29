# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F411 bare-metal flight controller firmware for a quadcopter. Uses DShot protocol for motor control, LSM6DSL IMU for attitude estimation (6660 Hz ODR), and PID control for stabilization.

## Build System

The project uses CMake with the ARM GCC toolchain.

**Build commands:**
```bash
# Using Python build script (recommended)
python build.py              # Debug build
python build.py --clean      # Clean + rebuild
python build.py --release    # Release build

# Manual CMake
mkdir build && cd build
cmake ..
make
```

**Flash to device:**
```bash
cd build
make flash                   # Uses OpenOCD with ST-Link
```

**Debug:**
```bash
cd build
make debug                   # Starts OpenOCD + GDB
```

## Architecture

**Main control loop (src/main.c):**
- State machine: INIT → DISARMED → ARMING → FLYING → EMERGENCY_STOP
- Runs at IMU data-ready interrupt rate (6660 Hz)
- Control flow: IMU update → PID update → motor output

**IMU subsystem (src/imu.c, src/imu.h):**
- LSM6DSL sensor via SPI (configured for 6660 Hz ODR, ±500°/s gyro, ±16g accel)
- Complementary filter for attitude estimation from gyro + accelerometer
- Provides roll/pitch/yaw attitude and angular velocities
- Data ready signal on PA0 interrupt (EXTI0)

**PID control (src/pid.c, src/pid.h):**
- Three independent PID controllers for roll, pitch, yaw
- Outputs clamped to prevent integral windup
- Takes setpoints and IMU measurements, produces motor corrections
- Each axis has separate Kp, Ki, Kd gains and output limits

**Motor control (src/motor_control.c, src/dshot.c):**
- 4 motors controlled via DShot600 protocol over DMA/PWM
- Motor mixing for quadcopter X configuration (in drive_motors() function)
- Throttle range: 1-2000 (DShot values)
- Each motor uses separate DMA stream for concurrent transmission

**Hardware peripherals:**
- SPI1: IMU communication (LSM6DSL)
- I2C1: Magnetometer (LIS3MDL, not currently used)
- TIM2/TIM3/TIM4/TIM5: DMA-backed PWM for DShot motor signals
- UART2: Debug output
- PC13: Onboard LED

## Important Implementation Details

**Fixed timestep:**
- System uses `FIXED_DT = 1.0f / 6660.0f` (~150 μs) synchronized to IMU data-ready
- Never use variable `dt` calculations - IMU ODR is constant

**Motor numbering:**
- motor1: North-East (front-right)
- motor2: North-West (front-left)
- motor3: South-East (back-right)
- motor4: South-West (back-left)

**DShot protocol:**
- Each frame: 16 bits (11-bit throttle + 1-bit telemetry + 4-bit CRC)
- Buffer double-buffering prevents DMA race conditions
- Motor commands: 0 (disarmed), 48 (special command), 1-2000 (throttle)

## File Organization

- `src/` - Application source code (.c and .h files)
- `inc/` - STM32 CMSIS headers and peripheral definitions
- `linker/STM32F411CEUx_FLASH.ld` - Linker script (128KB flash, 128KB RAM)
- `cmake/stm32f411.cmake` - ARM toolchain configuration
- `build/` - Build output (gitignored)
- `vendor/` - Third-party libraries (STM32CubeF4)

## Current State

Based on git status, active development areas:
- DShot motor control refinement (src/dshot.c)
- PID tuning (src/pid.c, src/pid.h)
- Main control loop state machine (src/main.c)

PID gains are currently set to 0.0 (tuning not yet completed).
