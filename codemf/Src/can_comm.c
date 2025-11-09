//
// Created by 21481 on 2025/4/26.
//


#include "can_comm.h"
#include "main.h"
#include "math.h"
#include "stdbool.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;





volatile float angle = 0;
volatile float speed = 0;

//8个电机的参数声明
struct xiaomi_motor xiaomimotors[8];

//电机总数
int8_t num_xiaomimotors = 8;

/**
  * @brief  Converts a float to an unsigned int, given range and number of bits
  * @param
  * @retval
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;

    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
  * @brief  converts unsigned int to float, given range and number of bits
  * @param
  * @retval
  */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
  * @brief  CAN接口初始化
  * @param
  * @retval
  */
void CanComm_Init(void)
{
    CAN_FilterTypeDef   sCAN_Filter;

    sCAN_Filter.FilterBank = 0;                         /* 指定将被初始化的过滤器 */
    sCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;     /* 过滤模式为屏蔽位模式 */
    sCAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;    /* 指定滤波器的规模 */
    sCAN_Filter.FilterIdHigh = 0x0000;
    sCAN_Filter.FilterIdLow = 0x0000;
    sCAN_Filter.FilterMaskIdHigh = 0x0000;
    sCAN_Filter.FilterMaskIdLow = 0x0000;
    sCAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sCAN_Filter.FilterActivation = ENABLE;              /* 启用或禁用过滤器 */
    sCAN_Filter.SlaveStartFilterBank = 0;               /* 选择启动从过滤器组 */

    HAL_CAN_ConfigFilter(&hcan1, &sCAN_Filter);
    HAL_CAN_Start(&hcan1);               /* 开启CAN通信 */
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);    /* 开启挂起中断允许 */

    sCAN_Filter.SlaveStartFilterBank = 14;
    sCAN_Filter.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &sCAN_Filter);
    HAL_CAN_Start(&hcan2);               /* 开启CAN通信 */
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);    /* 开启挂起中断允许 */


}





/**
  * @brief  Can总线发送控制参数
  * @param
  * @retval
  */
  //p:期望位置 v:期望速度 kp: kd: t:前馈扭矩
  //禁止发除电流外的参数，固件bug
void CanComm_SendControlPara(struct xiaomi_motor xiaomimotor_para)
{
    float f_p = 0 ;
    float f_v = 0 ;
    float f_kp = 0 ;
    float f_kd = 0 ;
    float f_t = xiaomimotor_para.give_tor;

    uint16_t p, v, kp, kd, t;//最终发送，经过转换的
    uint8_t buf[8];

    /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);

    /* 根据协议，对float参数进行转换 */
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);

    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    buf[0] = p>>8;
    buf[1] = p&0xFF;
    buf[2] = v>>4;
    buf[3] = ((v&0xF)<<4)|(kp>>8);
    buf[4] = kp&0xFF;
    buf[5] = kd>>4;
    buf[6] = ((kd&0xF)<<4)|(t>>8);
    buf[7] = t&0xff;

    /* 通过CAN接口把buf中的内容发送出去 */
    CanTransmit(buf, sizeof(buf),xiaomimotor_para.can_channel,xiaomimotor_para.can_id);
}



/*小米电机的特殊帧，用于使能、零点设置、失能*/
void CanComm_ControlCmd(uint8_t cmd , struct xiaomi_motor xiaomimotor_cmd)
{
    uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    switch(cmd)
    {
        case CMD_MOTOR_MODE://
            buf[7] = 0xFC;
            break;

        case CMD_RESET_MODE://
            buf[7] = 0xFD;
            break;

        case CMD_ZERO_POSITION://设置零点
            buf[7] = 0xFE;
            break;

        default:
            return; /* 直接退出函数 */
    }
    CanTransmit(buf, sizeof(buf), xiaomimotor_cmd.can_channel, xiaomimotor_cmd.can_id);
}





/* 把buf中的内容通过CAN接口发送出去 */
static void CanTransmit(uint8_t *buf, uint8_t len ,uint8_t can_channel , uint8_t motor_id)
{
    if(can_channel == 0x01)
    {
        CAN_TxHeaderTypeDef can1_TxHead;             /**!< can通信发送协议头 */
        uint32_t can1_TxMailbox;

        if((buf != NULL) && (len != 0))
        {
            can1_TxHead.StdId    = motor_id;     /* 指定标准标识符，该值在0x00-0x7FF */
            can1_TxHead.IDE      = CAN_ID_STD;       /* 指定将要传输消息的标识符类型 */
            can1_TxHead.RTR      = CAN_RTR_DATA;     /* 指定消息传输帧类型 */
            can1_TxHead.DLC      = len;              /* 指定将要传输的帧长度 */

            if(HAL_CAN_AddTxMessage(&hcan1, &can1_TxHead, buf, (uint32_t *)&can1_TxMailbox) == HAL_OK )
            {
            }
        }
    }
    if(can_channel == 0x02)
    {
        CAN_TxHeaderTypeDef can2_TxHead;             /**!< can通信发送协议头 */
        uint32_t can2_TxMailbox;

        if((buf != NULL) && (len != 0))
        {
            can2_TxHead.StdId    = motor_id;     /* 指定标准标识符，该值在0x00-0x7FF */
            can2_TxHead.IDE      = CAN_ID_STD;       /* 指定将要传输消息的标识符类型 */
            can2_TxHead.RTR      = CAN_RTR_DATA;     /* 指定消息传输帧类型 */
            can2_TxHead.DLC      = len;              /* 指定将要传输的帧长度 */

            if(HAL_CAN_AddTxMessage(&hcan2, &can2_TxHead, buf, (uint32_t *)&can2_TxMailbox) == HAL_OK )
            {
            }
        }
    }


}


/**
  * @brief  CAN接口接收数据
  * @param
  * @retval
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

    if(hcan == &hcan1)
    {
        CAN_RxHeaderTypeDef can1_RxHead; /**!< can通信协议头 */
        uint8_t can1_data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can1_RxHead, can1_data);

        switch (can1_data[0])
        {
            case (0x01):
            case (0x02):
            case (0x03):
            case (0x04):
            {
                int p_int;
                int v_int;
                int t_int;

                p_int = (can1_data[1] << 8) | can1_data[2];
                v_int = (can1_data[3] << 4) | (can1_data[4] >> 4);
                t_int = ((can1_data[4] & 0xF) << 8) | can1_data[5];
                xiaomimotors[can1_data[0] - 0x01].last_angle = xiaomimotors[can1_data[0] - 0x01].return_angle ;
                xiaomimotors[can1_data[0] - 0x01].old_fifiltering_speed = xiaomimotors[can1_data[0] - 0x01].fifilter_compute_speed ;
                xiaomimotors[can1_data[0] - 0x01].return_angle = uint_to_float(p_int, P_MIN, P_MAX, 16);
                xiaomimotors[can1_data[0] - 0x01].return_speed = uint_to_float(v_int, V_MIN, V_MAX, 12);
                xiaomimotors[can1_data[0] - 0x01].return_tor = uint_to_float(t_int, T_MIN, T_MAX, 12);

                float current_angle = xiaomimotors[can1_data[0] - 0x01].return_angle;
                float last_angle = xiaomimotors[can1_data[0] - 0x01].last_angle;
                float diff = current_angle - last_angle;
                if (diff > 12.0f) {
                    diff -= 25.0f;
                } else if (diff < -12.0f) {
                    diff += 25.0f;
                }
                xiaomimotors[can1_data[0] - 0x01].new_no_filtering_speed = diff / 0.001f ;

                xiaomimotors[can1_data[0] - 0x01].fifilter_compute_speed =
                        xiaomimotors[can1_data[0] - 0x01].new_no_filtering_speed
                        * xiaomimotors[can1_data[0] - 0x01].alpha_speed
                        + (1.0f - xiaomimotors[can1_data[0] - 0x01].alpha_speed)
                          * xiaomimotors[can1_data[0] - 0x01].old_fifiltering_speed ;

                break;
            }
            default:
            {
                break;
            }
        }
    }
    else if(hcan == &hcan2)
    {

        CAN_RxHeaderTypeDef can2_RxHead; /**!< can通信协议头 */
        uint8_t can2_data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can2_RxHead, can2_data);

        switch (can2_data[0])
        {
            case (0x01):
            case (0x02):
            case (0x03):
            case (0x04):
            {
                int p_int;
                int v_int;
                int t_int;

                p_int = (can2_data[1] << 8) | can2_data[2];
                v_int = (can2_data[3] << 4) | (can2_data[4] >> 4);
                t_int = ((can2_data[4] & 0xF) << 8) | can2_data[5];
                xiaomimotors[can2_data[0] + 0x03].last_angle = xiaomimotors[can2_data[0] + 0x03].return_angle ;
                xiaomimotors[can2_data[0] + 0x03].old_fifiltering_speed = xiaomimotors[can2_data[0] + 0x03].fifilter_compute_speed ;
                xiaomimotors[can2_data[0] + 0x03].return_angle = uint_to_float(p_int, P_MIN, P_MAX, 16);
                xiaomimotors[can2_data[0] + 0x03].return_speed = uint_to_float(v_int, V_MIN, V_MAX, 12);
                xiaomimotors[can2_data[0] + 0x03].return_tor = uint_to_float(t_int, T_MIN, T_MAX, 12);

                float current_angle = xiaomimotors[can2_data[0] + 0x03].return_angle;
                float last_angle = xiaomimotors[can2_data[0] + 0x03].last_angle;
                float diff = current_angle - last_angle;
                if (diff > 12.0f) {
                    diff -= 25.0f;
                } else if (diff < -12.0f) {
                    diff += 25.0f;
                }
                xiaomimotors[can2_data[0] + 0x03].new_no_filtering_speed = diff / 0.001f ;

                xiaomimotors[can2_data[0] + 0x03].fifilter_compute_speed =
                        xiaomimotors[can2_data[0] + 0x03].new_no_filtering_speed
                        * xiaomimotors[can2_data[0] + 0x03].alpha_speed
                        + (1.0f - xiaomimotors[can2_data[0] + 0x03].alpha_speed)
                        * xiaomimotors[can2_data[0] + 0x03].old_fifiltering_speed ;



                break;
            }
            default:
            {
                break;
            }
        }
    }





}








