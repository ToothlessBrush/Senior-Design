# components.cmake

# component ARM::CMSIS:CORE@6.1.1
add_library(ARM_CMSIS_CORE_6_1_1 INTERFACE)
target_include_directories(ARM_CMSIS_CORE_6_1_1 INTERFACE
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${CMSIS_PACK_ROOT}/ARM/CMSIS/6.2.0/CMSIS/Core/Include"
)
target_compile_definitions(ARM_CMSIS_CORE_6_1_1 INTERFACE
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_link_libraries(ARM_CMSIS_CORE_6_1_1 INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)

# component Keil::Device:Startup@2.1.0
add_library(Keil_Device_Startup_2_1_0 OBJECT
  "${SOLUTION_ROOT}/FlightController/RTE/Device/STM32F411CEUx/startup_stm32f411xe.s"
  "${SOLUTION_ROOT}/FlightController/RTE/Device/STM32F411CEUx/system_stm32f4xx.c"
)
target_include_directories(Keil_Device_Startup_2_1_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${CMSIS_PACK_ROOT}/Keil/STM32F4xx_DFP/2.4.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
)
target_compile_definitions(Keil_Device_Startup_2_1_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Keil_Device_Startup_2_1_0 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Keil_Device_Startup_2_1_0 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)
set(COMPILE_DEFINITIONS
  STM32F411xE
  _RTE_
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${CMSIS_PACK_ROOT}/Keil/STM32F4xx_DFP/2.4.0/MDK/Device/Source/ARM/STM32F4xx_OTP.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)
set(COMPILE_DEFINITIONS
  STM32F411xE
  _RTE_
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${SOLUTION_ROOT}/FlightController/RTE/Device/STM32F411CEUx/startup_stm32f411xe.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)
