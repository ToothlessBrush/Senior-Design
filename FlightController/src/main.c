#include "flight_controller.h"
#include "scheduler.h"
#include "system.h"
#include "systick.h"
#include "uart.h"

int main(void) {
    SystemClock_Config_100MHz_HSE();
    systick_init();
    delay_ms(500);

    uart_init(UART_INSTANCE_1, 420000); // CRSF receiver
    uart_init(UART_INSTANCE_2, 115200); // LoRa / BT tuning link

    fc_init(); // init IMU, PID, CRSF, LoRa, LED

    Task tasks[] = {
        {task_imu_pid, 0, 0},      // every loop, self-gated by imu_data_ready
        {task_crsf_service, 0, 0}, // constantly check crsf
        {task_config_service, 50, 0}, // 20 Hz — PID tuning + config via LoRa/BT
        {task_led, 500, 0},           // 2 Hz — LED status
    };

    init_scheduler(tasks, sizeof(tasks) / sizeof(tasks[0]));

    while (1) {
        run_scheduler();
    }
}
