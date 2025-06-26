//
// Created by 21481 on 2025/4/16.
//
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "can_comm.h"
#include "motor_control.h"


void MOTOR_CONTROL_TASK()
{
    osDelay(3000);
    CanComm_ControlCmd(CMD_MOTOR_MODE);
    osDelay(100);
    CanComm_SendControlPara(0,0,0,0,0);
    osDelay(100);
    CanComm_ControlCmd(CMD_ZERO_POSITION);
    osDelay(10);

    while (1)
    {

        CanComm_SendControlPara(0, 0, 0, 0.0f, 0.00f);//只用最后一个
        osDelay(10);





        osDelay(1);
    }
}

