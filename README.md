# Senior Design Drone Firmware

Bare-metal quadcopter flight controller for the STM32F411CEU6 (Black Pill).

**Hardware:** LSM6DSL IMU (SPI), ELRS receiver (CRSF/UART1), LoRa RYLR998 or HC-05 BT (UART2), DSHOT motors (x4), HC-SR04 sonar, optical flow sensor (UART6).

**Software:** Cooperative task scheduler, PID controller, sensor fusion, CRSF parser, LoRa/BT config protocol.

## Dependencies

**Arch Based**
```bash
sudo pacman -S arm-none-eabi-gcc cmake openocd gdb make

```
**Ubuntu Based**
```bash
sudo apt install gcc-arm-none-eabi cmake openocd gdb-multiarch make
```

**Windows**
- Install ARM GNU Toolchain and add to PATH
- Install CMake, Make, and OpenOCD
- Or use WSL with the Debian/Ubuntu instructions above

## Build

```bash
mkdir build && cd build
cmake ..
make

Flash (st-flash)

make flash

Debug (openocd)

make debug
```
