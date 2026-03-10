#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdint.h>

typedef enum {
    FC_ARMED = (1 << 0),
    FC_FAILSAFE = (1 << 1),
    FC_SIGNAL_OK = (1 << 2),
} FcStateFlags;

extern uint8_t fc_state;

#define IS_ARMED(s) ((s) & FC_ARMED)
#define IS_FAILSAFE(s) ((s) & FC_FAILSAFE)
#define IS_SIGNAL_OK(s) ((s) & FC_SIGNAL_OK)

void fc_init(void);
void arm(void);
void disarm(void);

void task_imu_pid(void);
void task_led(void);
void task_crsf_service(void);
void task_config_service(void);

#endif // FLIGHT_CONTROLLER_H
