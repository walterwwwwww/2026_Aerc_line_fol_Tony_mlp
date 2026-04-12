#ifndef APP_BUTTON_H
#define APP_BUTTON_H

/*
 * app_button.h
 * 按键与 LED 模块头文件
 *
 * ====== 按键复用方案 (3 按键映射原 4 按键功能) ======
 *
 *   原始按键         原始功能             STM32 映射           新按键
 *   ─────────────────────────────────────────────────────────────────
 *   anniu1 (P13_0)   启动/停止           BTN1 (PD3)           按键1
 *   anniu2 (P13_1)   进入校准模式         BTN2 (PD2)           按键2
 *   anniu3 (P13_2)   退出校准模式         BTN3 (PD1)           按键3
 *   anniu4 (P13_3)   (未使用)            —                    —
 *
 * 按键电平:
 *   原始: GPIO 下拉, 按下=HIGH(1), 释放=LOW(0)
 *   STM32: GPIO 上拉, 按下=LOW(0), 释放=HIGH(1)
 *   → 在 app_get_button() 中取反, 使 anniu_zhuangtai[][] 行为与原始一致
 *
 * 按键状态机说明:
 *   anniu_zhuangtai[i][0] = 当前状态 (经取反: 1=按下, 0=释放, 与原始一致)
 *   anniu_zhuangtai[i][1] = 上一次状态
 *   检测下降沿 (释放瞬间): [0]==0 && [1]==1
 *
 * LED 映射:
 *   LED1 = PC12  ← 原 P12_1   (运行指示)
 *   LED2 = PC11  ← 原 P12_0   (按键指示)
 *   LED3 = PC10  ← 原 P11_2   (校准指示)
 *   LED4 = PA15  ← 原 P11_1   (备用)
 *   注意: 原始 LED 低电平点亮 (set=1→灭, set=0→亮)
 *         STM32 根据实际硬件调整, 默认假设高电平点亮
 *
 * 拨码开关:
 *   原始有 boma (P07_2), STM32 版本未配置拨码引脚
 *   boma_zhuangtai[] 保留但始终为 0
 */

#include "main.h"

/* ---- 按键数量 ---- */
#define BTN_COUNT   3       // STM32 只有 3 个按键 (原始 4 个, 第4个未使用)

/* ---- 导出变量 (保持原始数组大小以兼容) ---- */
extern int anniu_zhuangtai[4][2];   // [按键编号][0=当前/1=上次], 仅 [0..2] 有效
extern int boma_zhuangtai[2];       // 拨码开关 (保留, 始终 0)
extern int anniu_start;

/* ---- 函数声明 ---- */
void app_button_init(void);         // GPIO 已由 CubeMX 初始化, 此函数清零状态
void app_get_button(void);          // 读取按键状态 (1ms 调用, 含取反)
void app_LED_set(int i, int j);     // 设置 LED: i=1~4, j=0→灭, j=1→亮

#endif /* APP_BUTTON_H */
