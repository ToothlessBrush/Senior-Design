#include <stdint.h>
enum State {
    FC_ARMED = (1 << 0),
    FC_FAILSAFE = (1 << 1),
    LORA_CONNECTED = (1 << 2),
};

uint8_t state = 0;

#define IS_ARMED(state) ((state) & FC_ARMED)
#define IS_CONNECTED(state) ((state) & LORA_CONNECTED)

void task_imu_pid(void);
void task_led(void);
void task_crsf_check_armed(void);
