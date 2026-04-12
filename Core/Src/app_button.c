#include "app_common.h"

/*
 * app_button.c
 * 按键与 LED 模块
 *
 * 按键电平适配:
 *   原始 (逐飞): 下拉模式, 按下=HIGH(1), 释放=LOW(0)
 *   STM32:       上拉模式, 按下=LOW(0),  释放=HIGH(1)
 *   → 读取 GPIO 后取反, 使 anniu_zhuangtai[][] 行为与原始一致
 *     即: anniu_zhuangtai[i][0] = 1 表示按下, 0 表示释放
 */

int anniu_zhuangtai[4][2] = {0};   // 保留原始数组大小 [4], 只用 [0..2]
int boma_zhuangtai[2] = {0};       // 拨码开关 (STM32 无此引脚, 始终 0)
int anniu_start = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键初始化
// 备注信息     GPIO 已由 CubeMX 初始化 (BTN1=PD3, BTN2=PD2, BTN3=PD1, 上拉)
//              此函数仅清零状态数组
//-------------------------------------------------------------------------------------------------------------------
void app_button_init(void)
{
    memset(anniu_zhuangtai, 0, sizeof(anniu_zhuangtai));
    memset(boma_zhuangtai, 0, sizeof(boma_zhuangtai));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取按键状态
// 备注信息     每 1ms 在 TIM7 中断中调用
//              读取 GPIO → 取反 → 存入 anniu_zhuangtai[][0]
//              上一次值移入 anniu_zhuangtai[][1]
//
//              按键映射:
//                anniu_zhuangtai[0] ← BTN1 (PD3) — 启动/停止
//                anniu_zhuangtai[1] ← BTN2 (PD2) — 进入校准
//                anniu_zhuangtai[2] ← BTN3 (PD1) — 退出校准
//                anniu_zhuangtai[3] ← (未接, 始终 0)
//-------------------------------------------------------------------------------------------------------------------
void app_get_button(void)
{
    /* 保存上一次状态 */
    boma_zhuangtai[1] = boma_zhuangtai[0];
    boma_zhuangtai[0] = 0;   // 无拨码开关

    for (int i = 0; i < 4; i++)
    {
        anniu_zhuangtai[i][1] = anniu_zhuangtai[i][0];
    }

    /*
     * 读取按键 GPIO 并取反
     * HAL_GPIO_ReadPin 返回: 未按=1 (上拉), 按下=0
     * 取反后: 未按=0, 按下=1  → 与原始逻辑一致
     */
    anniu_zhuangtai[0][0] = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3);   // BTN1
    anniu_zhuangtai[1][0] = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);   // BTN2
    anniu_zhuangtai[2][0] = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1);   // BTN3
    anniu_zhuangtai[3][0] = 0;                                       // 未接
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置 LED 状态
// 参数说明     i: LED 编号 (1~4)
//              j: 0=灭, 1=亮
// 备注信息     LED 引脚:
//                LED1 = PC12
//                LED2 = PC11
//                LED3 = PC10
//                LED4 = PA15
//              ⚠️ 原始代码中 set(i,0)=灭(GPIO=1), set(i,1)=亮(GPIO=0)
//                 即 LED 低电平点亮. STM32 默认也按此逻辑:
//                 j=0 → GPIO_SET(高) → 灭
//                 j=1 → GPIO_RESET(低) → 亮
//                 如实际硬件为高电平点亮, 请将 SET/RESET 反转
//-------------------------------------------------------------------------------------------------------------------
void app_LED_set(int i, int j)
{
    GPIO_TypeDef *port;
    uint16_t pin;

    switch (i)
    {
        case 1: port = GPIOC; pin = GPIO_PIN_12; break;   // LED1
        case 2: port = GPIOC; pin = GPIO_PIN_11; break;   // LED2
        case 3: port = GPIOC; pin = GPIO_PIN_10; break;   // LED3
        case 4: port = GPIOA; pin = GPIO_PIN_15; break;   // LED4
        default: return;
    }

    if (j == 0)
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);    // 灭 (高电平)
    else
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);  // 亮 (低电平)
}
