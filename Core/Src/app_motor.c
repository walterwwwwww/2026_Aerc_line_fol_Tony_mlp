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
 *   左电机: CH2(PA9)=正转(前进), CH1(PA8)=反转(后退)
 *   右电机: CH4(PA11)=正转(前进), CH3(PA10)=反转(后退)
 *
 * 死区补偿:
 *   原始代码在 pwm 输出时加 700 的死区补偿
 *   缩放后约 588, 此处使用 PWM_SCALE(700)
 *
 * ⚠️ 相比原版的变动 (为支持 VOFA+ 在线调参):
 *   原 #define A    (1000)   →  int motor_pid_limit = 1000;   PID 增量限幅
 *   原 #define C    (9300)   →  int motor_pwm_limit = 9300;   占空比限幅
 *   原 #define DEAD (700)    →  int motor_dead_zone = 700;    死区补偿
 *   宏改成全局变量后, VOFA+ 可以运行时修改, 不用烧录
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

/* ---- 限幅参数 (原为 #define, 改成变量以支持 VOFA 在线调整) ----
 *   motor_pid_limit — PID 每步增量限幅 (原宏 A)
 *   motor_pwm_limit — 占空比最大值    (原宏 C)
 *   motor_dead_zone — 死区补偿        (原宏 DEAD)
 */
int motor_pid_limit = 1000;
int motor_pwm_limit = 9300;
int motor_dead_zone = 700;

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
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);

    /* 蜂鸣器 (TIM3_CH3, PB0) — 初始关闭 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);

    pwm_ = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     速度环 PID 计算
// 参数说明     mode: 0=关闭速度环, 1=打开速度环
// 备注信息     算法与原始 nyy_motor_get() 完全一致
//              ⚠️ 变动: 原来 #define A/C 的限幅改用变量 motor_pid_limit / motor_pwm_limit
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

        if (PIDL >  motor_pid_limit) PIDL =  motor_pid_limit;
        if (PIDL < -motor_pid_limit) PIDL = -motor_pid_limit;

        pwm_ += (int)PIDL;
        if (pwm_ >  motor_pwm_limit) pwm_ =  motor_pwm_limit;
        if (pwm_ < -motor_pwm_limit) pwm_ = -motor_pwm_limit;
    }

    /* 叠加差速 (duty 来自角速度环) */
    pwml = (int)(pwm_ - duty);
    pwmr = (int)(pwm_ + duty);

    /* 占空比限幅 */
    if (pwml < -motor_pwm_limit) pwml = -motor_pwm_limit;
    if (pwml >  motor_pwm_limit) pwml =  motor_pwm_limit;
    if (pwmr < -motor_pwm_limit) pwmr = -motor_pwm_limit;
    if (pwmr >  motor_pwm_limit) pwmr =  motor_pwm_limit;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机 PWM 输出
// 参数说明     i: 1=正常输出, 0=全部停止
// 备注信息     所有 duty 值通过 PWM_SCALE 宏缩放到 TIM1 的 0~8399 范围
//              ⚠️ 变动: 原 #define DEAD/C 改用变量 motor_dead_zone / motor_pwm_limit
//-------------------------------------------------------------------------------------------------------------------
void app_motor_out(int i)
{
    if (i == 1)
    {
        app_LED_set(1, 0);  // 运行指示灯灭 (表示在运行)

        /* ---- 左电机 (CH1/CH2) ← pwmr ---- */
        int duty_l = pwmr;
        if (duty_l < -motor_pwm_limit) duty_l = -motor_pwm_limit;
        if (duty_l >  motor_pwm_limit) duty_l =  motor_pwm_limit;

        if (duty_l > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_SCALE(duty_l + motor_dead_zone));
        }
        else if (duty_l < 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_SCALE(-duty_l + motor_dead_zone));
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        }
        else  /* duty_l == 0: 不加死区, 电机停止 */
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        }

        /* ---- 右电机 (CH3/CH4) ← pwml ---- */
        int duty_r = pwml;
        if (duty_r < -motor_pwm_limit) duty_r = -motor_pwm_limit;
        if (duty_r >  motor_pwm_limit) duty_r =  motor_pwm_limit;

        if (duty_r > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_SCALE(duty_r + motor_dead_zone));
        }
        else if (duty_r < 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_SCALE(-duty_r + motor_dead_zone));
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
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
// 函数简介     负压风扇控制 (电调驱动无刷电机)
// 参数说明     k: 1=启动, 0=关闭
// 备注信息     TIM3_CH4 (PB1), PWM 周期由 TIM3 配置决定 (标准舵机/电调协议: 1000~2000μs)
//
//              启动时序 (电调必须按此顺序才能解锁并运转):
//                1) 1000μs  保持 1 秒 — 最低油门, 电调解锁 / 初始化
//                2) 1500μs  保持 3 秒 — 中油门, 电机开始运转并稳定
//                3) 1800μs          — 目标工作油门
//-------------------------------------------------------------------------------------------------------------------
#define FAN_THROTTLE_MIN     1000    // 最低油门 (电调解锁)
#define FAN_THROTTLE_MID     1500    // 中油门   (过渡启动)
#define FAN_THROTTLE_RUN     1800    // 工作油门 (目标转速)

/* ---- 安装测试 ---- */
#define MOTOR_TEST_STEP_MS  2000   /* 每步持续时间 (ms) */
#define MOTOR_TEST_PWM      2000   /* 测试占空比 (~24%), 足以转动又不会飞 */

static int motor_test_cnt = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     电机安装测试 — 依次驱动四路 H 桥, 验证接线方向
// 备注信息     在 TIM7 ISR 每 1ms 调用一次 (通过 app_START)
//              motortest 步骤:
//                1 = CH2   期望: 左轮前进
//                2 = CH1   期望: 左轮后退
//                3 = CH4   期望: 右轮前进
//                4 = CH3   期望: 右轮后退
//              每步 2 秒, 自动推进, 完成后 motortest=0
//              触发: BTN3 松开 (当 run==0 && jiaozhun==0)
//              LED4 闪烁表示测试进行中
//-------------------------------------------------------------------------------------------------------------------
void app_motor_install_test(void)
{
    if (motortest == 0) return;

    motor_test_cnt++;

    /* 时间到 → 推进到下一步 */
    if (motor_test_cnt >= MOTOR_TEST_STEP_MS)
    {
        motor_test_cnt = 0;
        motortest++;
        if (motortest > 4)
        {
            motortest = 0;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            app_LED_set(4, 0);
            return;
        }
    }

    /* 全部通道先清零, 再单独置对应步骤 */
    int pwm_val = PWM_SCALE(MOTOR_TEST_PWM + motor_dead_zone);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

    switch (motortest)
    {
        case 1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_val); break; /* 左前进 */
        case 2: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_val); break; /* 左后退 */
        case 3: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_val); break; /* 右前进 */
        case 4: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_val); break; /* 右后退 */
        default: break;
    }

    /* LED4 每 500ms 翻转一次 */
    app_LED_set(4, (motor_test_cnt % 1000) < 500 ? 1 : 0);
}

void app_fengshan(int k)
{
    if (k == 1)
    {
        /* ---- 第一步: 最低油门 1000μs, 保持 1 秒 ---- */
        /*   让电调检测到最低油门信号并完成解锁初始化       */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, FAN_THROTTLE_MIN);
        HAL_Delay(1000);

        /* ---- 第二步: 中油门 1500μs, 保持 3 秒 ---- */
        /*   电机起转并稳定, 为进入工作转速做准备          */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, FAN_THROTTLE_MID);
        HAL_Delay(3000);

        /* ---- 第三步: 切到目标油门 1800μs, 风扇进入工作状态 ---- */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, FAN_THROTTLE_RUN);
    }
    else
    {
        /* 关闭时回到最低油门, 不要给 0 (给 0 电调会丢信号并重新进入解锁流程) */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, FAN_THROTTLE_MIN);
    }
}