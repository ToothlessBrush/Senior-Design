# Core sources shared between the main firmware and all test targets.
# Tests link against this to get startup, clocks, systick, and utilities
# without pulling in application-specific code.
add_library(core STATIC
    startup_stm32f411xe.s
    system_stm32f4xx.c
    systick.c
    system.c
    utils.c
    protocol.c
    syscalls.c
)
target_link_libraries(core PUBLIC stm32f411)

# Main flight controller firmware
add_executable(FlightController
    main.c
    scheduler.c
    flight_controller.c
    uart.c
    i2c.c
    spi.c
    imu.c
    pid.c
    lora.c
    bluetooth.c
    motor_control.c
    optical_flow.c
    crsf.c
)
# To use LoRa instead of Bluetooth, uncomment the line below:
# target_compile_definitions(FlightController PRIVATE USE_LORA)
target_link_libraries(FlightController PRIVATE core)

add_custom_command(TARGET FlightController POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:FlightController>
            ${CMAKE_CURRENT_BINARY_DIR}/FlightController.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:FlightController>
    COMMENT "Generating FlightController.bin"
)

add_custom_target(flash
    COMMAND st-flash write ${CMAKE_CURRENT_BINARY_DIR}/FlightController.bin 0x08000000
    DEPENDS FlightController
    COMMENT "Flashing FlightController"
)

add_custom_target(debug
    COMMAND openocd -f interface/stlink.cfg -f target/stm32f4x.cfg &
    COMMAND sleep 2
    COMMAND arm-none-eabi-gdb ${CMAKE_CURRENT_BINARY_DIR}/FlightController
        -ex "target extended-remote localhost:3333"
        -ex "load"
        -ex "monitor reset halt"
    DEPENDS FlightController
    COMMENT "Starting debug session"
)
