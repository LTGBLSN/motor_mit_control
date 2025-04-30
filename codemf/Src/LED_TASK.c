//
// Created by 21481 on 2025/4/16.
//


#include "main.h"
#include "cmsis_os.h"


void LED_TASK()
{
    while (1)
    {
        HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin);
        osDelay(70);
        HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
        osDelay(70);
        HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
        osDelay(70);



        osDelay(1);
    }

}

