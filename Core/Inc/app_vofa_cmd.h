#ifndef __APP_VOFA_CMD_H__
#define __APP_VOFA_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * app_vofa_cmd.h
 * VOFA+ 运行时参数调整模块
 *
 * 目的:
 *   通过 USART2 RX (PD6) 接收 VOFA+ "命令控制" 面板发来的 ASCII 命令,
 *   在不烧录的情况下修改 PID 参数、限幅、巡航速度等, 极大加快调参速度
 *
 * 命令格式:
 *   NAME=VALUE\n           设置参数 (例: P_A=15.5)
 *   ?\n                    查询所有参数 (通过 USART2 回显)
 *   !\n                    紧急停车 (run=0, 清零所有积分)
 *
 * 支持的参数 (在 app_vofa_cmd.c 的 param_table 里定义):
 *   外环 PD:  P_tobeA, D_tobeA
 *   中环 PI:  P_A, I_A
 *   内环 PID: KP_L, KI_L, KD_L, KP_R, KI_R, KD_R
 *   速度设定: base_speed (巡航速度), speed (直接写入 speed_)
 *   限幅:    pid_limit, pwm_limit, dead_zone, duty_limit
 *   运行:    run, max_time
 *
 * 示例 (VOFA+ 命令面板):
 *   P_A=18.5\n           把角速度环 P 改成 18.5
 *   base_speed=60\n      巡航速度改成 60
 *   P_tobeA=600\n        外环 P 改成 600
 *   run=1\n              启动 (等效 BTN1)
 *   !\n                  紧急停车
 *   ?\n                  查询当前参数
 *
 * 硬件使用:
 *   复用 USART2 (和 VOFA+ 波形发送同一个串口)
 *   - TX (PD5): app_vofa.c 的 DMA1_Stream6 发送波形
 *   - RX (PD6): app_vofa_cmd.c 的 DMA1_Stream5 接收命令 + IDLE 中断
 *   TX/RX 硬件独立, 互不影响
 *
 * 使用流程:
 *   1. main.c 里 app_vofa_init() 之后调用 app_vofa_cmd_init()
 *   2. main.c 的 while(1) 里调用 app_vofa_cmd_process() (解析命令)
 *   3. stm32f4xx_it.c 的 USART2_IRQHandler 里先调 app_vofa_cmd_on_uart_irq()
 *   4. VOFA+ 打开"命令控制"面板, 输入命令 + 回车即可
 */

/* 公开 API */

/* 初始化: 配置 USART2 RX DMA + 使能 IDLE 中断. main() 里调用一次 */
void app_vofa_cmd_init(void);

/* 解析已收到的完整命令行. 在主循环 while(1) 里定期调用 */
void app_vofa_cmd_process(void);

/* 打印当前所有参数到 USART2 (响应 "?" 命令) */
void app_vofa_cmd_dump_params(void);

/* IDLE 中断处理. 由 stm32f4xx_it.c 的 USART2_IRQHandler 在 HAL 之前调用 */
void app_vofa_cmd_on_uart_irq(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_VOFA_CMD_H__ */