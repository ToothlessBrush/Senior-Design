#include "flight_controller.h"
#include "optical_flow.h"
#include "scheduler.h"
#include "system.h"
#include "systick.h"
#include "uart.h"

static char _buf[160];

int main(void) {
  SystemClock_Config_100MHz_HSE();
  systick_init();
  delay_ms(500);

  optical_flow_init();
  uart_init(UART_INSTANCE_2, 115200); // RNBT-B693 (RN-42) default baud rate

  // clang-format off
  Task tasks[] = {
    {task_imu_pid, 0, 0},         
    {task_crsf_service, 0, 0},
    {task_config_service, 50, 0}, 
    {task_led, 100, 0},
  };

  init_scheduler(tasks, sizeof(tasks) / sizeof(tasks[0]));

  while (1) {
    run_scheduler();
  }
}
