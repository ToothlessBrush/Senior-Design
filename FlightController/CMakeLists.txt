cmake_minimum_required(VERSION 3.16)

# Set toolchain before project declaration
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32f411.cmake)
project(FlightController C ASM)

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define target MCU
set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F411xE)

# Build target selection
set(BUILD_TARGET "main" CACHE STRING "Target to build: main or test name (e.g., test_optical_flow)")
message(STATUS "Building target: ${BUILD_TARGET}")

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Core sources - always included
set(CORE_SOURCES
    src/startup_stm32f411xe.s
    src/system_stm32f4xx.c
    src/systick.c
    src/system.c
    src/utils.c
    src/protocol.c
)

# Configure sources based on build target
if(BUILD_TARGET STREQUAL "main")
    # Main application sources
    set(SOURCES
        ${CORE_SOURCES}
        src/main.c
        src/uart.c
        src/i2c.c
        src/spi.c
        src/imu.c
        src/pid.c
        src/dshot.c
        src/lora.c
        src/motor_control.c
        src/optical_flow.c
    )
    set(EXECUTABLE_NAME "FlightController")
else()
    # Test target - check if test file exists
    set(TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/tests/${BUILD_TARGET}.c")
    if(EXISTS ${TEST_FILE})
        message(STATUS "Building test: ${BUILD_TARGET}")

        # Start with core sources and test file
        set(SOURCES
            ${CORE_SOURCES}
            ${TEST_FILE}
        )

        # Add dependencies based on test name
        # For optical flow test
        if(BUILD_TARGET STREQUAL "test_optical_flow")
            list(APPEND SOURCES
                src/uart.c
                src/lora.c
                src/optical_flow.c
            )
        endif()

        # Add more test-specific dependencies here as needed
        # if(BUILD_TARGET STREQUAL "test_something_else")
        #     list(APPEND SOURCES ...)
        # endif()

        set(EXECUTABLE_NAME "${BUILD_TARGET}")
    else()
        message(FATAL_ERROR "Test file not found: ${TEST_FILE}\nAvailable tests in tests/ directory")
    endif()
endif()

# Create executable
add_executable(${EXECUTABLE_NAME} ${SOURCES})

# Compiler definitions
target_compile_definitions(${EXECUTABLE_NAME} PRIVATE
    -D${MCU_MODEL}
    -D${MCU_FAMILY}
    -DARM_MATH_CM4
)

# Compiler options
target_compile_options(${EXECUTABLE_NAME} PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -fdata-sections
    -ffunction-sections
    -Wall
    -Wextra
    -g3                           # Always include debug symbols
    $<$<CONFIG:Debug>:-Og>        # Debug: Optimize for debugging
    $<$<CONFIG:Release>:-O3>      # Release: Maximum optimization
    $<$<CONFIG:Release>:-flto>    # Release: Link-time optimization
    -fno-math-errno               # No errno for cortex (faster math)
)


target_link_options(${EXECUTABLE_NAME} PRIVATE
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
    $<$<CONFIG:Release>:-flto>    # Link-time optimization for release
)

# Generate binary file
add_custom_command(TARGET ${EXECUTABLE_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE_NAME}> ${EXECUTABLE_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE_NAME}>
    COMMENT "Creating binary file and showing size info"
)

# Custom target to build binary explicitly
add_custom_target(bin
    DEPENDS ${EXECUTABLE_NAME}
    COMMENT "Building binary file"
)
add_custom_target(flash
    COMMAND openocd
        -f interface/stlink.cfg
        -f target/stm32f4x.cfg
        -c "init"
        -c "reset halt"
        -c "flash write_image erase ${EXECUTABLE_NAME}"
        -c "reset run"
        -c "shutdown"
    DEPENDS ${EXECUTABLE_NAME}
    COMMENT "Flashing the target with OpenOCD"
)
# Debug target - starts OpenOCD server and GDB
add_custom_target(debug
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f4x.cfg &
    COMMAND sleep 2
    COMMAND gdb ${EXECUTABLE_NAME} -ex "target extended-remote localhost:3333" -ex "load" -ex "monitor reset halt"
    DEPENDS ${EXECUTABLE_NAME}
    COMMENT "Starting debug session with OpenOCD and GDB"
)
