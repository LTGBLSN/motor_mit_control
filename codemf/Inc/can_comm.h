//
// Created by 21481 on 2025/4/26.
//

#ifndef MOTOR_MIT_CONTROL_CAN_COMM_H
#define MOTOR_MIT_CONTROL_CAN_COMM_H

#include "main.h"

#define CAN_SLAVE_ID        0x01

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03


#define P_MIN -12.5f    // Radians
#define P_MAX 12.5f
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))



struct xiaomi_motor{
    uint8_t can_id;//id
    uint8_t can_channel;//can1还是can2
    uint8_t state;//状态
    float last_angle;//上一次的位置
    float return_angle;//当前位置
    float return_speed;//回传速度
    float alpha_speed ;//一阶低通滤波系数
    float new_no_filtering_speed ;//新算出来的未滤波速度
    float old_fifiltering_speed ;
    float fifilter_compute_speed;//能用的速度
    float return_tor;//回传力矩
    float Tmos;//mos温度
    float Tcoil;//线圈温度
    float give_tor;//发送的力矩
};


extern struct xiaomi_motor xiaomimotors[8];
extern int8_t num_xiaomimotors ;

void CanComm_Init(void);
static void CanTransmit(uint8_t *buf, uint8_t len ,uint8_t can_channel , uint8_t motor_id);
void CanComm_SendControlPara(struct xiaomi_motor xiaomimotor_para);
void CanComm_ControlCmd(uint8_t cmd , struct xiaomi_motor xiaomimotor_cmd);


#endif //MOTOR_MIT_CONTROL_CAN_COMM_H
