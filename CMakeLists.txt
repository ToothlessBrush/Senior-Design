cmake_minimum_required(VERSION 3.16)

# Toolchain must be set before project()
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/stm32f411.cmake)
project(FlightController C ASM)

# Default to Release so sqrtf inlines to vsqrt.f32; at -O0 the linker pulls
# libm's wf_sqrt.o wrapper which references __errno and fails to link.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Interface library — ARM Cortex-M4 flags inherited by all targets via `core`
add_library(stm32f411 INTERFACE)

target_compile_definitions(stm32f411 INTERFACE
    STM32F411xE
    STM32F4xx
    ARM_MATH_CM4
)

target_include_directories(stm32f411 INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(stm32f411 INTERFACE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -fdata-sections
    -ffunction-sections
    -Wall
    -Wextra
    -g3
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-flto>
    -fno-math-errno
)

target_link_options(stm32f411 INTERFACE
    -T${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F411CEUx_FLASH.ld
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    -specs=nosys.specs
    -Wl,--start-group
    -lm
    -lc
    -Wl,--end-group
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    $<$<CONFIG:Release>:-flto>
)

add_subdirectory(src)
add_subdirectory(tests)
