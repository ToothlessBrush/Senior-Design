# STM32F411 Project

A simple LED blink project for the STM32F411 microcontroller using bare metal C programming.

## Prerequisites

### Required Software
- **ARM GCC Toolchain**: `arm-none-eabi-gcc`
- **CMake**: Version 3.16 or higher
- **OpenOCD**: For debugging and flashing
- **GDB**: For debugging (regular `gdb` or `gdb-multiarch`)
- **Make**: Build system

### Installation (Arch Linux)
```bash
sudo pacman -S arm-none-eabi-gcc cmake openocd gdb make
```

### Installation (Ubuntu/Debian)
```bash
sudo apt install gcc-arm-none-eabi cmake openocd gdb-multiarch make
```

## Hardware Requirements
- STM32F411 development board (e.g., STM32F411CE "Black Pill")
- ST-Link V2 programmer/debugger
- USB cables for power and programming

``

## Building the Project

### 1. Create Build Directory
```bash
mkdir build && cd build
```

### 2. Configure CMake
```bash
cmake ..
```

### 3. Build
```bash
make
```

This creates:
- `stm32f411_project` - ELF file with debug symbols (used for flashing and debugging)

## Flashing the Firmware

### Method 1: Using CMake Target (Recommended)
```bash
make flash
```

### Method 2: Manual OpenOCD Command
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program stm32f411_project verify reset exit"
```

## Debugging

### Start Debug Session
```bash
make debug
```

This will:
1. Start OpenOCD debug server on port 3333
2. Launch GDB connected to the target
3. Load the program and halt at reset

### Manual Debug Setup

**Terminal 1 - Start OpenOCD:**
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
```

**Terminal 2 - Connect GDB:**
```bash
gdb stm32f411_project
(gdb) target extended-remote localhost:3333
(gdb) load
(gdb) monitor reset halt
```

### Essential GDB Commands

#### Code Navigation
```bash
(gdb) layout next            # Show source code in TUI mode
(gdb) break main             # Set breakpoint at main function
(gdb) break main.c:25        # Set breakpoint at specific line
(gdb) continue               # Continue execution
(gdb) step                   # Step into functions
(gdb) next                   # Step over functions
```

#### Target Control
```bash
(gdb) monitor reset halt     # Reset and halt the MCU
(gdb) monitor reset          # Reset and run
(gdb) load                   # Load program to target
(gdb) continue               # Continue execution
```

#### Variable and Memory Inspection
```bash
(gdb) info locals            # Show local variables
(gdb) print variable_name    # Print specific variable
(gdb) print *pointer_name    # Dereference pointer
(gdb) print/x my_int         # Print in hexadecimal
(gdb) x/4x 0x20000000       # Examine 4 hex words at address
(gdb) info registers         # Show all registers
(gdb) print $r0              # Print specific register
```

#### Memory Addresses (STM32F411)
```bash
(gdb) x/8x 0x40020000        # GPIOA registers
(gdb) x/8x 0x40020800        # GPIOC registers (onboard LED)
(gdb) x/8x 0x20000000        # RAM start
(gdb) x/8x 0x08000000        # Flash start (vector table)
```

#### Exit Debug Session
```bash
(gdb) quit                   # Exit GDB
# Ctrl+C in OpenOCD terminal to stop server
```

## Troubleshooting

### Build Issues
- **Missing toolchain**: Install ARM GCC toolchain
- **CMake errors**: Check CMake version (requires 3.16+)
- **Missing headers**: Ensure `inc/` directory exists with required headers

### Flashing Issues
- **ST-Link not detected**: Check USB connection and drivers
- **Permission denied**: Add user to `dialout` group or use `sudo`
- **Target not found**: Verify ST-Link connection to STM32

### Debug Issues
- **Cannot connect to target**: Check ST-Link connection and OpenOCD output
- **No debugging symbols**: Ensure build includes `-g3` flag
- **GDB crashes**: Try `gdb-multiarch` instead of regular `gdb`

## Hardware Connections

### ST-Link to STM32F411
| ST-Link Pin | STM32F411 Pin | Function |
|-------------|---------------|----------|
| SWDIO       | PA13          | SWD Data |
| SWCLK       | PA14          | SWD Clock|
| GND         | GND           | Ground   |
| 3.3V        | 3.3V          | Power    |

## Project Features

- **Bare metal C programming** - No HAL or external libraries
- **LED blink on PC13** - Simple GPIO control demonstration
- **Full debug support** - Source-level debugging with GDB
- **Optimized build system** - CMake with ARM toolchain
- **Memory efficient** - Minimal resource usage

## Expanding the Project

To add more source files:
1. Add `.c` files to `src/` directory
2. Add corresponding `.h` files to `inc/` directory  
3. Update `SOURCES` list in `CMakeLists.txt`:
   ```cmake
   set(SOURCES
       src/main.c
       src/new_file.c
       src/startup_stm32f411xx.s
   )
   ```

## License

This project is provided as-is for educational and development purposes.
