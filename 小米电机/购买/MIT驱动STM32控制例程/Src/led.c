#include "led.h"
#include "gpio.h"

volatile uint32_t pre_run_tick = 0;
volatile uint32_t pre_can_tick = 0;


void LED_RUN(void)
{
    uint32_t led_tick = HAL_GetTick();
    if((led_tick - pre_run_tick) >= 100)
    {
        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
        pre_run_tick = led_tick;
    }
}

void LED_CAN(void)
{
    uint32_t led_tick = HAL_GetTick();
     if((led_tick - pre_can_tick) >= 50)
     {
        HAL_GPIO_TogglePin(LED_CAN_GPIO_Port, LED_CAN_Pin);
        pre_can_tick = led_tick;
     }
}



