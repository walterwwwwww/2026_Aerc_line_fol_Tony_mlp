#include "app_common.h"
#include "app_pidtest.h"

/*
 * app_pidtest.c
 * PID 调参辅助模块
 *
 * 使用方法:
 *   1) 开机, BTN3 短按切换测试模式 (LED4 闪烁次数指示模式)
 *   2) BTN1 按一下开始测试, 再按一下停止
 *   3) VOFA+ 观察波形对比 (已复用原有通道)
 *
 *   模式 0: 关闭测试, BTN1 就是正常启动/停止
 *   模式 1: 速度环    — 车轮架空, speedl/speedr 跟随 speed_ (方波: 0 ↔ 40)
 *   模式 2: 角速度环  — 车放平地(无线), jiaosudu 跟随 jiaosudutobe (方波: -60 ↔ +60 °/s)
 *   模式 3: 外环      — 车放赛道上, 手动横向偏移车身看修正响应
 *                        实际还是让车跑, 只是固定 speed_=30 看修正
 *
 * VOFA+ 通道复用 (已有):
 *   模式 1 看:  CH5 (speed_) vs CH6 (speedl) / CH7 (speedr)
 *   模式 2 看:  CH1 (jiaosudutobe) vs CH2 (jiaosudu)
 *   模式 3 看:  CH0 (adc_error) vs CH1 (jiaosudutobe) vs CH2 (jiaosudu)
 *
 * ⚠️ 在测试模式下, app_START() 中的正常巡线控制被接管:
 *    app_pidtest_is_active() 返回 1 时, main state machine 不要再写 speed_ / jiaosudutobe
 */

/* ==================== 参数 (可调) ==================== */

/* --- 模式 1: 速度环 --- */
#define SPEED_TEST_HIGH     40      /* 方波高电平 */
#define SPEED_TEST_LOW      0       /* 方波低电平 */
#define SPEED_TEST_PERIOD   3000    /* 方波周期 ms (3 秒切换一次) */

/* --- 模式 2: 角速度环 --- */
#define ANGRATE_TEST_HIGH   60.0f   /* °/s */
#define ANGRATE_TEST_LOW   -60.0f
#define ANGRATE_TEST_PERIOD 2000    /* 2 秒切换一次 */
#define ANGRATE_BASE_SPEED  30      /* 测试角速度环时用的前进速度 */

/* --- 模式 3: 外环 --- */
#define OUTER_TEST_SPEED    30      /* 外环测试时的前进速度 */

/* ==================== 变量 ==================== */

PidTestMode pidtest_mode     = PIDTEST_OFF;
int         pidtest_running  = 0;
float       pidtest_setpoint = 0;

static int  mode_tick        = 0;    /* 模式内部计时 */
static int  btn3_last        = 0;    /* BTN3 边沿检测 */
static int  btn1_last        = 0;    /* BTN1 边沿检测 */

/* LED 指示模式: 每 1500ms 周期内闪 N 次, N=mode */
#define LED_BLINK_CYCLE   1500
#define LED_BLINK_WIDTH   150

/* ==================== 内部函数 ==================== */

static void led_indicate_mode(void)
{
    /* 模式 0: LED4 常灭 */
    if (pidtest_mode == PIDTEST_OFF) {
        app_LED_set(4, 0);
        return;
    }

    /* 模式 N: 每 LED_BLINK_CYCLE ms 内闪 N 次 */
    int t = mode_tick % LED_BLINK_CYCLE;
    int n = (int)pidtest_mode;

    int on = 0;
    for (int i = 0; i < n; i++) {
        int start = i * LED_BLINK_WIDTH * 2;
        int end   = start + LED_BLINK_WIDTH;
        if (t >= start && t < end) { on = 1; break; }
    }
    app_LED_set(4, on);
}

/* 切换到下一个模式 */
static void cycle_mode(void)
{
    pidtest_running = 0;       /* 切模式时自动停止当前测试 */
    pidtest_setpoint = 0;
    speed_ = 0;
    jiaosudutobe = 0;

    pidtest_mode = (PidTestMode)(((int)pidtest_mode + 1) % PIDTEST_MODE_MAX);
    mode_tick = 0;
}

/* ==================== 各模式生成激励 ==================== */

static void run_speed_test(void)
{
    /* 方波: 0 → 40 → 0 → 40 ... */
    int phase = (mode_tick / SPEED_TEST_PERIOD) & 1;
    speed_ = phase ? SPEED_TEST_HIGH : SPEED_TEST_LOW;
    pidtest_setpoint = (float)speed_;

    /* 测速度环时不转向 */
    jiaosudutobe = 0;

    /* 仍然调用环路, 但外环不参与 */
    angle_ring(1);
    app_motor_get(1);
    app_motor_out(1);
}

static void run_angrate_test(void)
{
    /* 方波: +60 → -60 → +60 ... (左右交替转) */
    int phase = (mode_tick / ANGRATE_TEST_PERIOD) & 1;
    jiaosudutobe = phase ? ANGRATE_TEST_HIGH : ANGRATE_TEST_LOW;
    pidtest_setpoint = jiaosudutobe;

    /* 给一个固定前进速度, 不然 angle_ring 低速分支走不对 */
    speed_ = ANGRATE_BASE_SPEED;

    angle_ring(1);
    app_motor_get(1);
    app_motor_out(1);
}

static void run_outer_test(void)
{
    /* 外环测试: 让 app_get_speed() 正常计算, 但固定前进速度 */
    /* 这样你可以手动推车、横向移车, 看 jiaosudutobe 的响应 */
    app_get_speed();
    speed_ = OUTER_TEST_SPEED;  /* 覆盖 app_get_speed 里的 speed_=40 */
    pidtest_setpoint = jiaosudutobe;

    angle_ring(1);
    app_motor_get(1);
    app_motor_out(1);
}

/* ==================== 公共接口 ==================== */

void app_pidtest_init(void)
{
    pidtest_mode     = PIDTEST_OFF;
    pidtest_running  = 0;
    pidtest_setpoint = 0;
    mode_tick        = 0;
    btn3_last        = 0;
    btn1_last        = 0;
}

int app_pidtest_is_active(void)
{
    /* 只有在模式非 0 且正在跑时, 才接管控制 */
    return (pidtest_mode != PIDTEST_OFF) && pidtest_running;
}

void app_pidtest_tick(void)
{
    mode_tick++;

    /* ---- BTN3 下降沿: 切换测试模式 ---- */
    int btn3_now = anniu_zhuangtai[2][0];
    if (btn3_now == 1 && btn3_last == 0) {
        cycle_mode();
    }
    btn3_last = btn3_now;

    /* LED 指示当前模式 */
    led_indicate_mode();

    /* ---- 模式 0: 完全不干预, 交给 app_START() ---- */
    if (pidtest_mode == PIDTEST_OFF) {
        pidtest_running = 0;
        return;
    }

    /* ---- 模式 1~3: BTN1 下降沿切换运行状态 ---- */
    int btn1_now = anniu_zhuangtai[0][0];
    if (btn1_now == 1 && btn1_last == 0) {
        pidtest_running = !pidtest_running;
        mode_tick = 0;   /* 重新计时 */

        if (!pidtest_running) {
            /* 停止: 电机归零 */
            speed_ = 0;
            jiaosudutobe = 0;
            duty = 0;
            app_motor_get(0);
            app_motor_out(0);
        }
    }
    btn1_last = btn1_now;

    /* ---- 运行中: 按当前模式生成激励 ---- */
    if (pidtest_running) {
        switch (pidtest_mode) {
            case PIDTEST_SPEED:   run_speed_test();   break;
            case PIDTEST_ANGRATE: run_angrate_test(); break;
            case PIDTEST_OUTER:   run_outer_test();   break;
            default: break;
        }
    } else {
        /* 未运行时, 保持电机停止 */
        app_motor_out(0);
    }
}