//
// Created by 21481 on 2025/4/16.
//

#include "cmsis_os.h"
#include "uart_printf.h"
#include "can_comm.h"

void DEBUG_PRINT_TASK()
{
    while (1)
    {
        float speed = can_get_speed();
        float angle = can_get_angle();
        usart6_printf("%f,%f \r\n", speed,angle);
        osDelay(10);

        osDelay(1);
//
    }
}

