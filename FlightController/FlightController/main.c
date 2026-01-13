#include "RTE_Components.h"
#include <stdint.h>
#include CMSIS_device_header
#include <stm32f411xe.h>

#include "dshot.h"
#include "motor_control.h"
#include"systick.h"


int main() {

    SystemClock_Config_100MHz_HSE();
    InitMotors();
    StartMotors();
    // for(volatile int i = 0; i < 10000000; i++); // delay
    // for (int i = 48; i < 1000; i+=5)
    // {
    //     SetMotorThrottle(motor1, i);
    //     SetMotorThrottle(motor2, i);
    //     SetMotorThrottle(motor3, i);
    //     SetMotorThrottle(motor4, i);
    //     for (volatile int j = 0; j < 100000; j++); // delay
    // }
    SetMotorThrottle(motor1, 0);
    SetMotorThrottle(motor2, 250);
    SetMotorThrottle(motor3, 500);
    SetMotorThrottle(motor4, 1000);
    // for(volatile int i = 0; i < 100000000; i++); // delay
    //     SetMotorThrottle(motor1, 100);
    //     for(volatile int i = 0; i < 10000000; i++); // delay
    //     SetMotorThrottle(motor2, 100);
    //     for(volatile int i = 0; i < 10000000; i++); // delay
    //     SetMotorThrottle(motor3, 100);
    //     for(volatile int i = 0; i < 10000000; i++); // delay
    //     SetMotorThrottle(motor4, 100);
    //     for(volatile int i = 0; i < 10000000; i++); // delay


    while(1)
    {
        for(volatile int i = 0; i < 100000000; i++); // delay
        SetMotorThrottle(motor1, 500);
        for(volatile int i = 0; i < 10000000; i++); // delay
        SetMotorThrottle(motor2, 500);
        for(volatile int i = 0; i < 10000000; i++); // delay
        SetMotorThrottle(motor3, 500);
        for(volatile int i = 0; i < 10000000; i++); // delay
        SetMotorThrottle(motor4, 500);
        
        
        for(volatile int i = 0; i < 100000000; i++); // delay
        SetMotorThrottle(motor1, 1000);
        for(volatile int i = 0; i < 10000000; i++); // delay
        SetMotorThrottle(motor2, 1000);
        for(volatile int i = 0; i < 10000000; i++); // delay
        SetMotorThrottle(motor3, 250);
        for(volatile int i = 0; i < 10000000; i++); // delay
        SetMotorThrottle(motor4, 250);
    }
}
