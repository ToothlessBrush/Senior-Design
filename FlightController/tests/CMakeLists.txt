# add_fw_test(TEST_NAME [extra_src_files...])
#
# Creates a flashable firmware binary from tests/<TEST_NAME>.c plus any
# extra source files needed. Also adds a flash_<TEST_NAME> target.
#
# Example:
#   add_fw_test(test_uart1  ${SRC}/uart.c)
#   add_fw_test(test_optical_flow  ${SRC}/uart.c ${SRC}/lora.c ${SRC}/optical_flow.c)
function(add_fw_test TEST_NAME)
    set(SRC ${CMAKE_SOURCE_DIR}/src)

    add_executable(${TEST_NAME} ${TEST_NAME}.c ${ARGN})
    target_link_libraries(${TEST_NAME} PRIVATE core)

    add_custom_command(TARGET ${TEST_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary
                $<TARGET_FILE:${TEST_NAME}>
                ${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TEST_NAME}>
        COMMENT "Generating ${TEST_NAME}.bin"
    )

    add_custom_target(flash_${TEST_NAME}
        COMMAND st-flash write ${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME}.bin 0x08000000
        DEPENDS ${TEST_NAME}
        COMMENT "Flashing ${TEST_NAME}"
    )
endfunction()

# ── Unit tests (host-native, separate toolchain) ─────────────────────────────
add_custom_target(unit_tests
    COMMAND ${CMAKE_COMMAND} -S ${CMAKE_CURRENT_SOURCE_DIR}/unit
                             -B ${CMAKE_CURRENT_BINARY_DIR}/unit
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR}/unit
    COMMAND ctest --test-dir ${CMAKE_CURRENT_BINARY_DIR}/unit --output-on-failure
    COMMENT "Building and running host unit tests"
    USES_TERMINAL
)

# ── Firmware tests ────────────────────────────────────────────────────────────
set(SRC ${CMAKE_SOURCE_DIR}/src)

add_fw_test(test_optical_flow
    ${SRC}/uart.c
    ${SRC}/lora.c
    ${SRC}/optical_flow.c
)
