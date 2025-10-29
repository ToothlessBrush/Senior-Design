cmake_minimum_required(VERSION 3.16)

# Set toolchain before project declaration
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32f411.cmake)
project(FlightController C ASM)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Define target MCU
set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F411xE)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Source files
set(SOURCES
    src/main.c
    src/startup_stm32f411xe.s
    src/system_stm32f4xx.c
    src/systick.c
    src/uart.c
    src/i2c.c
    src/spi.c
    src/imu.c
    src/pid.c
    src/dshot.c
    src/lora.c
    src/motor_control.c
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Compiler definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE
    -D${MCU_MODEL}
    -D${MCU_FAMILY}
    -DARM_MATH_CM4
)

# Compiler options
target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -fdata-sections
    -ffunction-sections
    -Wall
    -Wextra
    -Og
    -g3
    -fno-math-errno # no errno for cortex
)


target_link_options(${PROJECT_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F411CEUx_FLASH.ld
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -Wl,--start-group
    -lm
    -lc
    -lnosys
    -Wl,--end-group
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# Generate binary file
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Creating binary file and showing size info"
)

# Custom target to build binary explicitly
add_custom_target(bin
    DEPENDS ${PROJECT_NAME}
    COMMENT "Building binary file"
)
add_custom_target(flash
    COMMAND openocd 
        -f interface/stlink.cfg 
        -f target/stm32f4x.cfg 
        -c "init" 
        -c "reset halt" 
        -c "flash write_image erase ${PROJECT_NAME}" 
        -c "reset run" 
        -c "shutdown"
    DEPENDS ${PROJECT_NAME}
    COMMENT "Flashing the target with OpenOCD"
)

# Debug target - starts OpenOCD server and GDB
add_custom_target(debug
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f4x.cfg &
    COMMAND sleep 2
    COMMAND gdb ${PROJECT_NAME} -ex "target extended-remote localhost:3333" -ex "load" -ex "monitor reset halt"
    DEPENDS ${PROJECT_NAME}
    COMMENT "Starting debug session with OpenOCD and GDB"
)
