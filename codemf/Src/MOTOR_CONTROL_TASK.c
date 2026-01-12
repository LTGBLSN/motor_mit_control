//
// Created by 21481 on 2025/4/16.
//
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "can_comm.h"


void MOTOR_CONTROL_TASK()
{
    osDelay(3000);//必要，等待电机初始化完成才可通讯

    CanComm_ControlCmd(CMD_MOTOR_MODE,xiaomimotors[4]);//电机1使能
    osDelay(100);
    CanComm_ControlCmd(CMD_MOTOR_MODE,xiaomimotors[5]);//电机2使能
    osDelay(100);
    CanComm_ControlCmd(CMD_MOTOR_MODE,xiaomimotors[6]);//电机3使能
    osDelay(100);
    xiaomimotors[4].give_tor = 0.0f ;
    CanComm_SendControlPara(xiaomimotors[4]);//电流归零

    while (1)
    {
        xiaomimotors[4].give_tor = 0.0f ;
        CanComm_SendControlPara(xiaomimotors[4]);
        xiaomimotors[5].give_tor = 0.0f ;
        CanComm_SendControlPara(xiaomimotors[5]);
        xiaomimotors[6].give_tor = 0.0f ;
        CanComm_SendControlPara(xiaomimotors[6]);
        osDelay(1);





        osDelay(1);
    }
}

