#include "motor_control.h"
#include "can_comm.h"
#include "math.h"

volatile uint32_t pre_PCtick = 0;       /**!< 位置控制前一个tick */
volatile uint32_t pre_VCtick = 0;       /**!< 速度控制前一个tick */
volatile uint32_t loop_counter = 0;
volatile float f_p = 0;

/* 在发送电机位置零前，需要把电机的所有控制参数设置为0 */
static void ZeroPosition(void)
{
    CanComm_ControlCmd(CMD_MOTOR_MODE);
    HAL_Delay(100);
    CanComm_SendControlPara(0,0,0,0,0);
    HAL_Delay(100);
}


/* 启动电机控制 */
void MotorControl_Start(void)
{
    ZeroPosition();
    CanComm_ControlCmd(CMD_ZERO_POSITION);
    loop_counter = 0;
    pre_PCtick = HAL_GetTick();
    pre_VCtick = HAL_GetTick();
}

void MotorControl_Stop(void)
{
    CanComm_ControlCmd(CMD_RESET_MODE);
}

/**
  * @brief  电机位置控制
  * @retval false :完成了一个周期的控制
            true：没有完成一个周期的控制
  */
bool MotorControl_PositionHandler(void)
{
    float t;
    
    uint32_t tick = HAL_GetTick();
    
    if((tick - pre_PCtick) >= 10)      /**!< 10ms进行一次控制 */
    {
        pre_PCtick = tick;
        loop_counter++;
        t = 0.01*loop_counter;      /**!< 定时器的时钟滴答为10ms */
        
        if(t < 3)
        {
            f_p = -20.0f+20.0f*cos(t);
        }
        else if((t > 3)&&(t < 6))
        {
            f_p = -60.0f+20*cos(t-3);
        }
        
        if(t > 6)
        {
            CanComm_ControlCmd(CMD_RESET_MODE);     /**!< 电机进入RESET模式 */
            loop_counter = 0;                       /**!< 控制count清零 */
            return false;                           /**!< 完成整个电机周期的控制 */
        }
        
        CanComm_SendControlPara(f_p, 0, 5, 0.2, 0);
    }
    return true;
}


/**
  * @brief  速度闭环控制
  * @retval false :完成了一个周期的控制
            true：没有完成一个周期的控制
  */
bool MotorControl_velocityHandler(void)
{
    uint32_t tick = HAL_GetTick();  
    
    if((tick - pre_VCtick) >= 10)
    {
        pre_VCtick = tick;
        
        CanComm_SendControlPara(0, 1, 0, 1, 0.2);
    }
    return true;
}

