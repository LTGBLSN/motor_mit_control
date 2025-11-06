//
// Created by 21481 on 2025/4/16.
//
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "can_comm.h"


void MOTOR_CONTROL_TASK()
{
    osDelay(3000);

    CanComm_ControlCmd(CMD_MOTOR_MODE,xiaomimotors[0]);
    CanComm_ControlCmd(CMD_MOTOR_MODE,xiaomimotors[1]);
    osDelay(100);
    CanComm_SendControlPara(xiaomimotors[0]);
    CanComm_SendControlPara(xiaomimotors[1]);

    while (1)
    {
        xiaomimotors[0].give_tor = 0.0f ;
        xiaomimotors[1].give_tor = 0.0f ;
        CanComm_ControlCmd(CMD_MOTOR_MODE,xiaomimotors[0]);
        osDelay(1);//±ØÒª
        CanComm_ControlCmd(CMD_MOTOR_MODE,xiaomimotors[1]);

        CanComm_SendControlPara(xiaomimotors[0]);
        CanComm_SendControlPara(xiaomimotors[1]);
        osDelay(10);





        osDelay(1);
    }
}

