//
// Created by 21481 on 2025/4/16.
//

#include "cmsis_os.h"
#include "uart_printf.h"
void DEBUG_PRINT_TASK()
{
    while (1)
    {

        usart1_printf("hello word \r\n");
        osDelay(10);

        osDelay(1);
//
    }
}

