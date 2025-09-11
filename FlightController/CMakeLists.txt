cmake_minimum_required(VERSION 3.16)

# Set toolchain before project declaration
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32f411.cmake)

project(stm32f411_project C ASM)

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

# Source files (only main.c for now, others can be added later)
set(SOURCES
    src/main.c
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Compiler definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE
    -D${MCU_MODEL}
    -D${MCU_FAMILY}
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
)

# Linker options
target_link_options(${PROJECT_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F411CEUx_FLASH.ld
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -lc
    -lm
    -lnosys
    -Wl,-Map=${PROJECT_NAME}.map,--cref
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# Post-build steps to create HEX and BIN files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJDUMP} -St $<TARGET_FILE:${PROJECT_NAME}> > ${PROJECT_NAME}.lst
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Building HEX and BIN files"
)

# Flash target
add_custom_target(flash
    COMMAND st-flash write ${PROJECT_NAME}.bin 0x8000000
    DEPENDS ${PROJECT_NAME}
    COMMENT "Flashing the target"
)

# Flash with st-util (alternative)
add_custom_target(flash_util
    COMMAND st-util &
    COMMAND sleep 2
    COMMAND arm-none-eabi-gdb -batch -ex "target remote localhost:4242" -ex "load" -ex "quit" ${PROJECT_NAME}
    COMMAND pkill st-util
    DEPENDS ${PROJECT_NAME}
    COMMENT "Flashing with st-util"
)
