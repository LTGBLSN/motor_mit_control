#include "button.h"
#include "multi_button.h"
#include "gpio.h"
#include "stdbool.h"

static struct Button bnt1;
static struct Button bnt2;

static int8_t bnt1_state = -1;
static int8_t bnt2_state = -1;

static uint8_t read_BNT1_GPIO(void)
{
    return HAL_GPIO_ReadPin(BNT1_GPIO_Port,BNT1_Pin);
}


static uint8_t read_BNT2_GPIO(void)
{
    return HAL_GPIO_ReadPin(BNT2_GPIO_Port,BNT2_Pin);
}

/* 单按 */
static void BNT1_SINGLE_Click_Handler(void *bnt)
{
    bnt1_state = SINGLE_CLICK;
}
static void BNT2_SINGLE_Click_Handler(void *bnt)
{
    bnt2_state = SINGLE_CLICK;
}

/* 双击 */
static void BNT1_DOUBLE_Click_Handler(void *bnt)
{
    bnt1_state = DOUBLE_CLICK;
}
static void BNT2_DOUBLE_Click_Handler(void *bnt)
{
    bnt2_state = DOUBLE_CLICK;
}

/* 长按 */
static void BNT1_LONG_PRESS_START_Handler(void *bnt)
{
    bnt1_state = LONG_RRESS_START;
}
static void BNT2_LONG_PRESS_START_Handler(void *bnt)
{
    bnt2_state = LONG_RRESS_START;
}

void Button_Init(void)
{
    button_init(&bnt1, read_BNT1_GPIO, 0);
    button_init(&bnt2, read_BNT2_GPIO, 0);
    
    button_attach(&bnt1, SINGLE_CLICK, BNT1_SINGLE_Click_Handler);
    button_attach(&bnt1, DOUBLE_CLICK, BNT1_DOUBLE_Click_Handler);
    button_attach(&bnt1, LONG_RRESS_START, BNT1_LONG_PRESS_START_Handler);
    
    button_attach(&bnt2, SINGLE_CLICK, BNT2_SINGLE_Click_Handler);
    button_attach(&bnt2, DOUBLE_CLICK, BNT2_DOUBLE_Click_Handler);
    button_attach(&bnt2, LONG_RRESS_START, BNT2_LONG_PRESS_START_Handler);
    
    button_start(&bnt1);
    button_start(&bnt2);
}

/* 读取BNT1的状态 */
int8_t get_bnt1_state(void)
{
    int8_t ret = bnt1_state;
    bnt1_state = -1;
    return ret;
}

/* 读取BNT2的状态 */
int8_t get_bnt2_state(void)
{
    int8_t ret = bnt2_state;
    bnt2_state = -1;
    return ret;
}





