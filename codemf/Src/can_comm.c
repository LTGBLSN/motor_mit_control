//
// Created by 21481 on 2025/4/26.
//


#include "can_comm.h"
#include "main.h"
#include "math.h"
#include "stdbool.h"


extern CAN_HandleTypeDef hcan1;





volatile float angle = 0;
volatile float speed = 0;

//8个电机的参数声明
struct xiaomi_motor xiaomimotors[8]= {
        {0x01,0x01,0,0,0,0,0,
                0,0,0,0},
        {0x02,0x01,0,0,0,0,0,
                0,0,0,0},
        {0x03,0x01,0,0,0,0,0,
                0,0,0,0},
        {0x04,0x01,0,0,0,0,0,
                0,0,0,0},


        {0x01,0x02,0,0,0,0,0,
                0,0,0,0},
        {0x02,0x02,0,0,0,0,0,
                0,0,0,0},
        {0x03,0x02,0,0,0,0,0,
                0,0,0,0},
        {0x04,0x02,0,0,0,0,0,
                0,0,0,0}
};
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
    sCAN_Filter.FilterScale = CAN_FILTERSCALE_16BIT;    /* 指定滤波器的规模 */
    sCAN_Filter.FilterIdHigh = 00;
    sCAN_Filter.FilterIdLow = 00;
    sCAN_Filter.FilterMaskIdHigh = 00;
    sCAN_Filter.FilterMaskIdLow = 00;
    sCAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sCAN_Filter.FilterActivation = ENABLE;              /* 启用或禁用过滤器 */
    sCAN_Filter.SlaveStartFilterBank = 0;               /* 选择启动从过滤器组 */

    HAL_CAN_ConfigFilter(&hcan1, &sCAN_Filter);
    HAL_CAN_Start(&hcan1);               /* 开启CAN通信 */
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);    /* 开启挂起中断允许 */

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
    CAN_TxHeaderTypeDef TxHead;             /**!< can通信发送协议头 */
    uint32_t canTxMailbox;

    if((buf != NULL) && (len != 0))
    {
        TxHead.StdId    = motor_id;     /* 指定标准标识符，该值在0x00-0x7FF */
        TxHead.IDE      = CAN_ID_STD;       /* 指定将要传输消息的标识符类型 */
        TxHead.RTR      = CAN_RTR_DATA;     /* 指定消息传输帧类型 */
        TxHead.DLC      = len;              /* 指定将要传输的帧长度 */

        if(HAL_CAN_AddTxMessage(&hcan1, &TxHead, buf, (uint32_t *)&canTxMailbox) == HAL_OK )
        {
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
    int p_int;
    int v_int;
    int t_int;
    CAN_RxHeaderTypeDef RxHead; /**!< can通信协议头 */
    uint8_t data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHead, data);

    switch (data[0])
    {
        case (0x01):
        case (0x02):
        case (0x03):
        case (0x04):
        {
            p_int = (data[1] << 8) | data[2];
            v_int = (data[3] << 4) | (data[4] >> 4);
            t_int = ((data[4] & 0xF) << 8) | data[5];
            xiaomimotors[data[0]-0x01].last_angle = xiaomimotors[data[0]-0x01].return_angle ;
            xiaomimotors[data[0]-0x01].return_angle = uint_to_float(p_int, P_MIN, P_MAX, 16);
            xiaomimotors[data[0]-0x01].return_speed = uint_to_float(v_int, V_MIN, V_MAX, 12);
            xiaomimotors[data[0]-0x01].return_tor = uint_to_float(t_int, T_MIN, T_MAX, 12);

            break;
        }
        default:
        {
            break;
        }
    }
}

float can_get_speed(void)
{
    return speed;
}

float can_get_angle(void)
{
    return angle;
}










