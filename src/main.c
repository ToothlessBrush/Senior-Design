#include "flight_controller.h"
#include "scheduler.h"
#include "system.h"
#include "systick.h"

static Task tasks[] = {
    {task_imu_pid, 0, 0},
    {task_crsf_service, 0, 0},
    {task_config_service, 50, 0},
    {task_led, 100, 0},
};

int main(void) {
    SystemClock_Config_100MHz_HSE();
    systick_init();
    delay_ms(500);

    fc_init();

    init_scheduler(tasks, sizeof(tasks) / sizeof(tasks[0]));

    while (1) {
        run_scheduler();
    }
}
