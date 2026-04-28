/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 速度环 PID 闭环测试 (单环, 无转向, 无巡线)
  ******************************************************************************
  *
  *   测试目的:
  *     给一个目标速度 → 速度环 PID 跑 → 看电机能不能跟得上
  *     主要是检查 PID 参数对 130 强磁是不是合理, 不再有震动
  *
  *   测试流程:
  *     1) 上电后 LED1 慢闪, 等 BTN1
  *     2) 按 BTN1 → 启动速度闭环, speed_ = base_speed (默认 50)
  *        - LED1 常亮表示测试中
  *        - 持续 TEST_DURATION_MS 后自动停止
  *     3) 测试期间随时按 BTN3 急停
  *     4) 测试结束 LED1 灭, 等下一次 BTN1
  *
  *   ★ VOFA+ 看什么 ★
  *
  *     主要看 CH5 (target = base_speed), CH6 (speedl), CH7 (speedr):
  *
  *     ✅ 健康响应: speedl/speedr 在 ~50~200ms 内爬升到接近 target,
  *                 之后稳定在 target 附近, 抖动 < ±10%
  *
  *     ❌ 一直震荡: speedl/speedr 在 0 附近来回跳, target 拉很高也不动
  *                 → PID 太凶, 把 KP_L/KP_R 减小, 或把 pid_limit 减小
  *
  *     ❌ 缓慢爬升不到位: speedl/speedr 升到一半就饱和
  *                       → KI_L/KI_R 太小, 或 pwm_limit 太小
  *
  *     ❌ 反向跑: speedl/speedr 一直是负的 → 编码器方向反了!
  *                 (先用上一版的方向测试程序确认)
  *
  *   ★ 实时调参 (不用重新烧录) ★
  *
  *     在 VOFA+ 命令面板里发命令 (每行一条, 以 \n 结尾):
  *       base_speed=80     改目标速度
  *       KP_L=10           改速度环 KP
  *       KI_L=0.2          改速度环 KI
  *       pid_limit=300     改 PID 每步增量限幅
  *       pwm_limit=8000    改 PWM 上限
  *       dead_zone=500     改死区
  *       ?                 列出所有可调参数
  *
  *   ⚠️ 车架请架空, 第一次测先把 base_speed 设小 (50 已经是默认)
  *   ⚠️ 转向环 (angle_ring) 不参与, duty 强制 = 0, 两轮命令值一样
  *   ⚠️ 巡线/陀螺仪/状态机全部不参与
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "app_common.h"
#include "app_vofa_cmd.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define TEST_DURATION_MS   5000     // 一次测试持续多久 (ms)
/* USER CODE END PD */

/* USER CODE BEGIN PV */
/* ⚠️ 这两个变量在 stm32f4xx_it_user.c 的 TIM7 ISR 里使用,
 *    用 volatile 防止编译器优化掉
 *
 *    speed_test_active = 1: ISR 里跑速度环 PID
 *                      = 0: ISR 里强制电机停转
 */
volatile int speed_test_active = 0;
/* USER CODE END PV */

/* USER CODE BEGIN PM */
#define LED_ON(P,N)      HAL_GPIO_WritePin(P, N, GPIO_PIN_RESET)
#define LED_OFF(P,N)     HAL_GPIO_WritePin(P, N, GPIO_PIN_SET)
#define LED_TOGGLE(P,N)  HAL_GPIO_TogglePin(P, N)
#define LED1_ON()   LED_ON (LED1_GPIO_Port, LED1_Pin)
#define LED1_OFF()  LED_OFF(LED1_GPIO_Port, LED1_Pin)
#define LED4_ON()   LED_ON (LED4_GPIO_Port, LED4_Pin)
#define LED4_OFF()  LED_OFF(LED4_GPIO_Port, LED4_Pin)
/* USER CODE END PM */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void wait_for_btn1(void);
static int  btn3_pressed(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void wait_for_btn1(void)
{
    while (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_SET)
    {
        LED_TOGGLE(LED1_GPIO_Port, LED1_Pin);
        HAL_Delay(150);
    }
    HAL_Delay(50);
    while (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_RESET)
        HAL_Delay(10);
    LED1_OFF();
}

static int btn3_pressed(void)
{
    return (HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin) == GPIO_PIN_RESET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();
    MX_TIM4_Init();
    MX_USART3_UART_Init();
    MX_TIM7_Init();
    /* USER CODE BEGIN 2 */

    app_button_init();
    app_encoder_init();
    app_motor_init();

    /* ⚠️ 速度环测试不需要这些, 跳过节省启动时间:
     *   - app_gyro_init()   陀螺仪 (启动校准 ~5s)
     *   - app_fengshan(1)   风扇   (启动 ~4s)
     *   - app_adc_init()    巡线 ADC
     */

    /* VOFA 必须开, 不然看不到波形 */
    app_vofa_init();
    app_vofa_cmd_init();    // 在线调参支持 (KP_L=10\n 这种)

    LED1_OFF();
    LED4_OFF();

    /* 启动 1ms 控制中断 (跑速度环 + 发 VOFA, 见 stm32f4xx_it_user.c) */
    HAL_TIM_Base_Start_IT(&htim7);

    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */

        /* ---- 等 BTN1 按下启动一轮测试 ---- */
        wait_for_btn1();

        /* ---- 启动闭环 ---- */
        speed_test_active = 1;
        LED1_ON();   // 测试中

        int elapsed = 0;
        int aborted = 0;
        while (elapsed < TEST_DURATION_MS)
        {
            /* 急停 */
            if (btn3_pressed())
            {
                aborted = 1;
                break;
            }

            /* 处理 VOFA+ 在线调参命令 (KP_L=xx, base_speed=xx 等)
             * 这是测试期间动态改 PID 的关键 */
            app_vofa_cmd_process();

            HAL_Delay(2);
            elapsed += 2;
        }

        /* ---- 关闭闭环, ISR 会让电机停转 ---- */
        speed_test_active = 0;
        LED1_OFF();

        /* 急停时 LED4 闪 3 秒, 提示是被中止的 */
        if (aborted)
        {
            for (int i = 0; i < 12; i++)
            {
                LED_TOGGLE(LED4_GPIO_Port, LED4_Pin);
                HAL_Delay(250);
            }
            LED4_OFF();
        }
        else
        {
            HAL_Delay(1000);
        }

        /* USER CODE END 3 */
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif