cmake_minimum_required(VERSION 3.27)

# Roots
include("../roots.cmake")

set(CONTEXT FlightController.Debug+STM32F411CEUx)
set(TARGET ${CONTEXT})
set(OUT_DIR "${SOLUTION_ROOT}/out/FlightController/STM32F411CEUx/Debug")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(HEX_FILE "FlightController.hex")
set(LD_MAP_FILE "FlightController.axf.map")
set(LD_SCRIPT "${SOLUTION_ROOT}/FlightController/RTE/Device/STM32F411CEUx/ac6_linker_script.sct.src")
set(LD_REGIONS "${SOLUTION_ROOT}/FlightController/RTE/Device/STM32F411CEUx/regions_STM32F411CEUx.h")
set(LD_SCRIPT_PP "${CMAKE_CURRENT_BINARY_DIR}/ac6_linker_script.sct")

# Processor Options
set(CPU Cortex-M4)
set(FPU SP_FPU)
set(BYTE_ORDER Little-endian)

# Toolchain config map
include("toolchain.cmake")

# Setup project
project(${CONTEXT} LANGUAGES C ASM)

# Compilation database
add_custom_target(database
  COMMAND ${CMAKE_COMMAND} -E make_directory "${OUT_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json" "${OUT_DIR}"
)
set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES})

# Setup context
add_executable(${CONTEXT})
set_target_properties(${CONTEXT} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "FlightController")
set_target_properties(${CONTEXT} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR})
add_library(${CONTEXT}_GLOBAL INTERFACE)

# Includes
target_include_directories(${CONTEXT} PUBLIC
  "${SOLUTION_ROOT}/FlightController/RTE/_Debug_STM32F411CEUx"
  "${CMSIS_PACK_ROOT}/ARM/CMSIS/6.2.0/CMSIS/Core/Include"
  "${CMSIS_PACK_ROOT}/Keil/STM32F4xx_DFP/2.4.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
)

# Defines
target_compile_definitions(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:C,CXX>:
    STM32F411xE
    _RTE_
  >
)

# Compile options
target_compile_options(${CONTEXT} PUBLIC
  $<$<COMPILE_LANGUAGE:ASM>:
    "SHELL:${ASM_CPU}"
    "SHELL:${ASM_FLAGS}"
    "SHELL:${ASM_BYTE_ORDER}"
    "SHELL:-gdwarf-5"
    "SHELL:-masm=auto"
  >
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_CPU}"
    "SHELL:${CC_FLAGS}"
    "SHELL:${CC_BYTE_ORDER}"
    "SHELL:-gdwarf-5"
    "SHELL:-std=gnu11"
    "SHELL:-Wno-macro-redefined"
    "SHELL:-Wno-pragma-pack"
    "SHELL:-Wno-parentheses-equality"
    "SHELL:-Wno-license-management"
  >
)

# Compile Options Abstractions
add_library(${CONTEXT}_ABSTRACTIONS INTERFACE)
cbuild_set_options_flags(CC "none" "on" "" "" CC_OPTIONS_FLAGS_${CONTEXT})
cbuild_set_options_flags(ASM "none" "on" "" "" ASM_OPTIONS_FLAGS_${CONTEXT})
target_compile_options(${CONTEXT}_ABSTRACTIONS INTERFACE
  $<$<COMPILE_LANGUAGE:C>:
    "SHELL:${CC_OPTIONS_FLAGS_${CONTEXT}}"
  >
  $<$<COMPILE_LANGUAGE:ASM>:
    "SHELL:${ASM_OPTIONS_FLAGS_${CONTEXT}}"
  >
)

# Add groups and components
include("groups.cmake")
include("components.cmake")

target_link_libraries(${CONTEXT} PUBLIC
  Group_Source_Files
  ARM_CMSIS_CORE_6_1_1
  Keil_Device_Startup_2_1_0
)

# Linker options
target_link_options(${CONTEXT} PUBLIC
  "SHELL:${LD_CPU}"
  "SHELL:${_LS}\"${LD_SCRIPT_PP}\""
  "SHELL:${LD_MAP}"
  "SHELL:--entry=Reset_Handler"
  "SHELL:--info summarysizes"
  "SHELL:--summary_stderr"
  "SHELL:--diag_suppress=L6314W"
)
set_target_properties(${CONTEXT} PROPERTIES LINK_DEPENDS ${LD_SCRIPT})

# Linker script pre-processing
add_custom_command(TARGET ${CONTEXT} PRE_LINK COMMAND ${CPP} ARGS ${CPP_ARGS_LD_SCRIPT} BYPRODUCTS ${LD_SCRIPT_PP})

# Hex Conversion
 add_custom_command(TARGET ${CONTEXT} POST_BUILD COMMAND ${CMAKE_OBJCOPY} ${ELF2HEX})
