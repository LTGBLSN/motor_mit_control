#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "stm32F0xx_hal.h"
#include "stdbool.h"

void MotorControl_Start(void);
void MotorControl_Stop(void);
bool MotorControl_PositionHandler(void);
bool MotorControl_velocityHandler(void);

#endif

