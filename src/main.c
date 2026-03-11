#include "flight_controller.h"
#include "scheduler.h"
#include "system.h"
#include "systick.h"

int main(void) {
    SystemClock_Config_100MHz_HSE();
    systick_init();
    delay_ms(500);

    fc_init();

    Task tasks[] = {
        {task_imu_pid, 0, 0},       // always update
        {task_crsf_service, 0, 0},  // always update
        {task_optical_flow, 20, 0}, // 50 Hz — drain UART6, update imu.velocity
        {task_config_service, 50, 0}, // only check periodically
        {task_led, 500, 0},           // led blinks at half a second
    };

    init_scheduler(tasks, sizeof(tasks) / sizeof(tasks[0]));

    while (1) {
        run_scheduler();
    }
}
