/**
 * @file test_optical_flow.c
 * @brief Test program for MTF-01 optical flow sensor
 *
 * Infinite loop test that checks for optical flow data and sends
 * telemetry via LoRa when data is received.
 */

#include "stm32f411xe.h"
#include "system.h"
#include "systick.h"
#include "uart.h"
#include "lora.h"
#include "optical_flow.h"
#include <stdio.h>
#include <string.h>

int main(void) {
    char log_buffer[128];
    uint32_t last_sequence = 0;
    uint32_t frame_count = 0;
    uint32_t last_report_time = 0;

    // Initialize system
    SystemClock_Config_100MHz_HSE();
    systick_init();

    // Wait for peripherals to power on
    delay_ms(500);

    // Initialize UART2 for LoRa communication
    uart_init(UART_INSTANCE_2, 115200);

    // Initialize LoRa module
    if (lora_init() == LORA_OK) {
        lora_send_string(1, "LOG:OPTICAL_FLOW_TEST_START");
    } else {
        // Hang if LoRa fails - we need it for debugging
        while (1) {
            delay_ms(1000);
        }
    }

    delay_ms(100);
    lora_send_string(1, "LOG:INITIALIZING_OPTICAL_FLOW");

    // Initialize optical flow sensor on UART6
    optical_flow_init();

    delay_ms(100);
    lora_send_string(1, "LOG:OPTICAL_FLOW_READY");
    lora_send_string(1, "LOG:RAW_HEX_DUMP_MODE");

    uint8_t hex_buffer[10];
    uint8_t hex_index = 0;

    // Infinite test loop - dump raw hex data
    while (1) {
        // Service LoRa to handle any incoming commands
        lora_service();

        // Read bytes from UART6 and buffer them
        while (uart_data_available(UART_INSTANCE_6) && hex_index < 10) {
            hex_buffer[hex_index++] = uart_receive_byte(UART_INSTANCE_6);
        }

        // Send buffered hex data
        if (hex_index > 0) {
            // Build hex string
            int offset = 0;
            offset += snprintf(log_buffer + offset, sizeof(log_buffer) - offset, "LOG:");
            for (uint8_t i = 0; i < hex_index; i++) {
                offset += snprintf(log_buffer + offset, sizeof(log_buffer) - offset,
                                   "%02X ", hex_buffer[i]);
            }

            lora_send_string(1, log_buffer);
            hex_index = 0;

            // Respect LoRa cooldown
            delay_ms(200);
        }

        // Small delay if no data
        if (!uart_data_available(UART_INSTANCE_6)) {
            delay_ms(10);
        }
    }

    return 0;
}
