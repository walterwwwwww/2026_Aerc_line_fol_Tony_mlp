#ifndef APP_MOTOR_H
#define APP_MOTOR_H

/*
 * app_motor.h
 * 电机驱动模块头文件
 *
 * 硬件映射 (TIM1, 168MHz / 8400 = 20kHz PWM):
 *   CH1 (PA8)  → 左电机正转   ← 原 P18_1
 *   CH2 (PA9)  → 左电机反转   ← 原 P18_3
 *   CH3 (PA10) → 右电机正转   ← 原 P18_7 (注意: 原右轮正反与接线有关)
 *   CH4 (PA11) → 右电机反转   ← 原 P18_6
 *
 * 风扇: TIM3_CH4 (PB1)       ← 原 P06_5
 *
 * PWM 占空比范围:
 *   原始: PWM_DUTY_MAX = 10000, Period 由逐飞库内部管理
 *   STM32: TIM1 ARR = 8399, 有效 compare 范围 0~8399
 *   缩放系数: STM32_CCR = original_duty * 8400 / 10000
 */

#include "main.h"

/* ---- PWM 参数 ---- */
#define PWM_PERIOD_TIM1     8400            // TIM1 ARR+1
#define PWM_SCALE(x)        ((int)((x) * PWM_PERIOD_TIM1 / 10000))  // 原始→STM32 缩放

/* ---- 导出变量 (与原始代码一致) ---- */
extern int   speedtobel, speedtober, motortest, speed_;
extern float KP_L, KI_L, KD_L;
extern float KP_R, KI_R, KD_R;
extern float L_err[3], R_err[3];
extern float L_sum, R_sum;
extern float PIDL, dl;
extern float PIDR, dr;
extern int   pwml, pwmr, pwm_;

/* ---- 函数声明 ---- */
void app_motor_init(void);
void app_motor_get(int mode);           // mode=0 关闭速度环, mode=1 打开
void app_motor_out(int i);              // i=1 输出 PWM, i=0 全部停止
void app_fengshan(int k);              // k=1 启动风扇, k=0 关闭

#endif /* APP_MOTOR_H */
