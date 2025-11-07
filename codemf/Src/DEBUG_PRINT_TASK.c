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
        usart6_printf("%f,%f \r\n",xiaomimotors[0].return_angle,
                      xiaomimotors[5].return_angle);

        osDelay(1);
//
    }
}

