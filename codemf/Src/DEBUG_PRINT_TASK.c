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
        usart6_printf("%f \r\n",
                      xiaomimotors[1].fifilter_compute_speed);

        osDelay(1);
//
    }
}

