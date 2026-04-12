#include "app_common.h"

/*
 * app_motor.c
 * 电机驱动模块
 *
 * 硬件:
 *   TIM1 CH1~CH4 (PA8~PA11), 20kHz PWM
 *   ARR = 8399, Prescaler = 0, APB2 Timer = 168MHz
 *   → 168MHz / (8399+1) = 20kHz ✓ (与原始 20kHz 一致)
 *
 * PWM 缩放:
 *   原始 PWM_DUTY_MAX = 10000, 本项目 ARR+1 = 8400
 *   所有 duty 值在输出时乘以 0.84 (通过 PWM_SCALE 宏)
 *
 * H桥接线:
 *   左电机: CH1(PA8)=正转, CH2(PA9)=反转
 *   右电机: CH3(PA10)=正转, CH4(PA11)=反转  (⚠️ 如方向反请交换 CH3/CH4)
 *
 * 死区补偿:
 *   原始代码在 pwm 输出时加 700 的死区补偿
 *   缩放后约 588, 此处使用 PWM_SCALE(700)
 */

/* ---- 速度环变量 (与原始完全一致) ---- */
int speedtobel = 0, speedtober = 0, motortest = 0;
int speed_ = 0;

float KP_L = 25, KI_L = 0.5f, KD_L = 0;
float KP_R = 25, KI_R = 0.5f, KD_R = 0;

float L_err[3] = {0, 0, 0};
float R_err[3] = {0, 0, 0};
float L_sum = 0;
float R_sum = 0;
float PIDL = 0, dl = 0;
float PIDR = 0, dr = 0;

int pwml = 0, pwmr = 0, pwm_ = 0;

/* ---- 限幅常量 (保持原始值, 输出时缩放) ---- */
#define A       (1000)      // PID 增量限幅
#define C       (9300)      // 占空比限幅
#define DEAD    (700)       // 死区补偿

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机 PWM 初始化
// 备注信息     启动 TIM1 所有 4 个 PWM 通道, 初始占空比 = 0
//              启动 TIM3_CH4 (PB1) 风扇 PWM
//-------------------------------------------------------------------------------------------------------------------
void app_motor_init(void)
{
    /* 启动 TIM1 PWM 输出 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // 左正转
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   // 左反转
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   // 右正转
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);   // 右反转

    /* 初始占空比全部为 0 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

    /* 启动风扇 PWM (TIM3_CH4, PB1) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

    /* 蜂鸣器 (TIM3_CH3, PB0) — 初始关闭 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

    pwm_ = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度环 PID 计算
// 参数说明     mode: 0=关闭速度环, 1=打开速度环
// 备注信息     算法与原始 nyy_motor_get() 完全一致
//-------------------------------------------------------------------------------------------------------------------
void app_motor_get(int mode)
{
    if (mode == 0)  // 关闭速度环
    {
        pwm_ = 0;
    }
    else if (mode == 1)  // 打开速度环
    {
        L_err[2] = L_err[1];
        L_err[1] = L_err[0];
        L_err[0] = ((speed_ - speedl) + (speed_ - speedr) - dl);
        L_sum = speed_ - speedl + speed_ - speedr;
        dl = speed_ - speedl + speed_ - speedr;

        /* 左右轮 PID */
        PIDL = (KP_L * L_err[0] + KI_L * L_sum + KD_L * (L_err[0] - L_err[1])) / 2;

        if (PIDL > A)  PIDL = A;
        if (PIDL < -A) PIDL = -A;

        pwm_ += (int)PIDL;
        if (pwm_ > C)  pwm_ = C;
        if (pwm_ < -C) pwm_ = -C;
    }

    /* 叠加差速 (duty 来自角速度环) */
    pwml = (int)(pwm_ - duty);
    pwmr = (int)(pwm_ + duty);

    /* 占空比限幅 */
    if (pwml < -C) pwml = -C;
    if (pwml >  C) pwml =  C;
    if (pwmr < -C) pwmr = -C;
    if (pwmr >  C) pwmr =  C;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机 PWM 输出
// 参数说明     i: 1=正常输出, 0=全部停止
// 备注信息     所有 duty 值通过 PWM_SCALE 宏缩放到 TIM1 的 0~8399 范围
//-------------------------------------------------------------------------------------------------------------------
void app_motor_out(int i)
{
    if (i == 1)
    {
        app_LED_set(1, 0);  // 运行指示灯灭 (表示在运行)

        /* ---- 左电机 (CH1/CH2) ← pwmr ---- */
        int duty_l = pwmr;
        if (duty_l < -C) duty_l = -C;
        if (duty_l >  C) duty_l =  C;

        if (duty_l > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_SCALE(duty_l + DEAD));
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        }
        else if (duty_l < 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_SCALE(-duty_l + DEAD));
        }
        else  /* duty_l == 0: 不加死区, 电机停止 */
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        }

        /* ---- 右电机 (CH3/CH4) ← pwml ---- */
        int duty_r = pwml;
        if (duty_r < -C) duty_r = -C;
        if (duty_r >  C) duty_r =  C;

        if (duty_r > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_SCALE(duty_r + DEAD));
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        }
        else if (duty_r < 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_SCALE(-duty_r + DEAD));
        }
        else  /* duty_r == 0: 不加死区, 电机停止 */
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
        }
    }
    else  /* 停止 */
    {
        app_LED_set(1, 1);  // 运行指示灯亮 (表示停止)
        pwm_ = 0;
        duty = 0;
        pwmr = 0;
        pwml = 0;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     风扇控制
// 参数说明     k: 1=启动(渐加速), 0=关闭
// 备注信息     TIM3_CH4 (PB1), Period=19999
//              原始最大占空比 fengli_max=3000/10000=30%
//              TIM3 ARR=19999, 等效 30% = 6000
//-------------------------------------------------------------------------------------------------------------------
void app_fengshan(int k)
{
    if (k == 1)
    {
        /* 渐进启动 (与原始行为一致) */
        for (int i = 0; i < 6000; i++)
        {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, i);
            /* 简易延时 ~4ms (非精确, 原始使用 system_delay_ms(4)) */
            HAL_Delay(4);
        }
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}