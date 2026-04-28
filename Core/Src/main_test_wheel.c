// /* USER CODE BEGIN Header */
// /**
//   ******************************************************************************
//   * @file           : main.c
//   * @brief          : Main program body  —  电机直接驱动测试版
//   ******************************************************************************
//   * @attention
//   *
//   *   测试模式说明:
//   *     1) TIM7 PID 中断已关闭, 不会有控制环抢 PWM
//   *     2) VOFA+ 已关闭, USART2 不占用
//   *     3) 风扇默认关闭 (太吵, 测电机用不上)
//   *     4) 上电后等待 BTN1 按下才开始转, 防止上电直接乱跑
//   *     5) 测试序列: 双正转 → 停 → 双反转 → 停 → 仅左转 → 仅右转 → 停 → 循环
//   *
//   *   ⚠️  车架请架空 (轮子离地), 出问题随时按 RESET
//   *   ⚠️  duty 默认只给 3000/10000 = 30%, 觉得不够再调 TEST_DUTY
//   *
//   ******************************************************************************
//   */
// /* USER CODE END Header */
// /* Includes ------------------------------------------------------------------*/
// #include "main.h"
// #include "dma.h"
// #include "spi.h"
// #include "tim.h"
// #include "usart.h"
// #include "gpio.h"

// /* Private includes ----------------------------------------------------------*/
// /* USER CODE BEGIN Includes */
// #include "app_common.h"
// // #include "app_vofa_cmd.h"   // 测试期间不用 VOFA
// /* USER CODE END Includes */

// /* Private typedef -----------------------------------------------------------*/
// /* USER CODE BEGIN PTD */

// /* USER CODE END PTD */

// /* Private define ------------------------------------------------------------*/
// /* USER CODE BEGIN PD */

// /* ---- 测试参数 ---- */
// #define TEST_DUTY        3000    // 测试占空比 (范围 0~10000, 30% = 3000)
// #define TEST_PHASE_MS    2000    // 每个阶段持续时间 ms

// /* USER CODE END PD */

// /* Private macro -------------------------------------------------------------*/
// /* USER CODE BEGIN PM */

// /* 直接控制 H 桥 PWM 的小工具宏, 让测试代码看起来更清爽
//  *   左电机 = CH1(正转) / CH2(反转)
//  *   右电机 = CH3(正转) / CH4(反转)
//  *   d 范围: 0~10000, PWM_SCALE 内部缩放到 TIM1 的 0~8399
//  */
// #define MOTOR_L_FWD(d)  do { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_SCALE(d)); \
//                              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); } while(0)
// #define MOTOR_L_REV(d)  do { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); \
//                              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_SCALE(d)); } while(0)
// #define MOTOR_L_STOP()  do { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); \
//                              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); } while(0)

// #define MOTOR_R_FWD(d)  do { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_SCALE(d)); \
//                              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); } while(0)
// #define MOTOR_R_REV(d)  do { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); \
//                              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_SCALE(d)); } while(0)
// #define MOTOR_R_STOP()  do { __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); \
//                              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); } while(0)

// #define MOTOR_ALL_STOP() do { MOTOR_L_STOP(); MOTOR_R_STOP(); } while(0)

// /* USER CODE END PM */

// /* Private variables ---------------------------------------------------------*/

// /* USER CODE BEGIN PV */

// /* USER CODE END PV */

// /* Private function prototypes -----------------------------------------------*/
// void SystemClock_Config(void);
// /* USER CODE BEGIN PFP */
// static void wait_for_btn1(void);
// /* USER CODE END PFP */

// /* Private user code ---------------------------------------------------------*/
// /* USER CODE BEGIN 0 */

// /* 等待 BTN1 按下 (PD3 低电平有效, 视具体硬件而定)
//  * 如果你板子上按键是高电平有效, 把判断改成 == GPIO_PIN_SET
//  */
// static void wait_for_btn1(void)
// {
//     /* LED1 闪烁提示 "等待按键" */
//     while (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_SET)
//     {
//         HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//         HAL_Delay(150);
//     }
//     /* 软件去抖 */
//     HAL_Delay(50);
//     /* 按键释放后再继续 (避免长按串到下一个流程) */
//     while (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == GPIO_PIN_RESET)
//     {
//         HAL_Delay(10);
//     }
//     HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);  // LED1 灭
// }

// /* USER CODE END 0 */

// /**
//   * @brief  The application entry point.
//   * @retval int
//   */
// int main(void)
// {

//   /* USER CODE BEGIN 1 */

//   /* USER CODE END 1 */

//   /* MCU Configuration--------------------------------------------------------*/

//   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//   HAL_Init();

//   /* USER CODE BEGIN Init */

//   /* USER CODE END Init */

//   /* Configure the system clock */
//   SystemClock_Config();

//   /* USER CODE BEGIN SysInit */

//   /* USER CODE END SysInit */

//   /* Initialize all configured peripherals */
//   MX_GPIO_Init();
//   MX_DMA_Init();
//   MX_SPI1_Init();
//   MX_SPI2_Init();
//   MX_TIM1_Init();
//   MX_TIM2_Init();
//   MX_TIM3_Init();
//   MX_USART2_UART_Init();
//   MX_TIM4_Init();
//   MX_USART3_UART_Init();
//   MX_TIM7_Init();
//   /* USER CODE BEGIN 2 */
//   app_button_init();
//   app_encoder_init();
//   app_motor_init();        // ← 关键: 只要这一句, PWM 通道就活了
//   app_gyro_init();
//   // app_fengshan(1);      // 测电机时关掉, 省得吵 (~24s 启动延迟也省了)
//   app_adc_init();

//   // app_vofa_init();      // 测电机不用 VOFA
//   // app_vofa_cmd_init();  // 测电机不用 VOFA 命令

//   // HAL_TIM_Base_Start_IT(&htim7);   // ⚠️ PID 中断关掉, 不让控制环抢 PWM

//   /* 上电先确保电机停转 */
//   MOTOR_ALL_STOP();

//   /* USER CODE END 2 */

//   /* Infinite loop */
//   /* USER CODE BEGIN WHILE */

//   /* ---- 等用户按 BTN1 才开始, 防止上电就乱跑 ---- */
//   wait_for_btn1();

//   while (1)
//   {
//     /* USER CODE END WHILE */

//     /* USER CODE BEGIN 3 */

//     /* ============ 阶段 1: 双轮正转 ============ */
//     HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);  // LED1 亮
//     MOTOR_L_FWD(TEST_DUTY);
//     MOTOR_R_FWD(TEST_DUTY);
//     HAL_Delay(TEST_PHASE_MS);

//     /* ============ 阶段 2: 全部停止 ============ */
//     HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);    // LED1 灭
//     MOTOR_ALL_STOP();
//     HAL_Delay(TEST_PHASE_MS);

//     /* ============ 阶段 3: 双轮反转 ============ */
//     HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);  // LED2 亮
//     MOTOR_L_REV(TEST_DUTY);
//     MOTOR_R_REV(TEST_DUTY);
//     HAL_Delay(TEST_PHASE_MS);

//     /* ============ 阶段 4: 全部停止 ============ */
//     HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);    // LED2 灭
//     MOTOR_ALL_STOP();
//     HAL_Delay(TEST_PHASE_MS);

//     /* ============ 阶段 5: 仅左轮正转 (用于确认左右接线) ============ */
//     HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);  // LED3 亮
//     MOTOR_L_FWD(TEST_DUTY);
//     MOTOR_R_STOP();
//     HAL_Delay(TEST_PHASE_MS);

//     /* ============ 阶段 6: 仅右轮正转 ============ */
//     HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);    // LED3 灭
//     HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);  // LED4 亮
//     MOTOR_L_STOP();
//     MOTOR_R_FWD(TEST_DUTY);
//     HAL_Delay(TEST_PHASE_MS);

//     /* ============ 阶段 7: 全部停止, 等待下一轮 ============ */
//     HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);    // LED4 灭
//     MOTOR_ALL_STOP();
//     HAL_Delay(TEST_PHASE_MS);

//     /* USER CODE END 3 */
//   }
// }

// /**
//   * @brief System Clock Configuration
//   * @retval None
//   */
// void SystemClock_Config(void)
// {
//   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//   /** Configure the main internal regulator output voltage
//   */
//   __HAL_RCC_PWR_CLK_ENABLE();
//   __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//   /** Initializes the RCC Oscillators according to the specified parameters
//   * in the RCC_OscInitTypeDef structure.
//   */
//   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//   RCC_OscInitStruct.PLL.PLLM = 8;
//   RCC_OscInitStruct.PLL.PLLN = 336;
//   RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//   RCC_OscInitStruct.PLL.PLLQ = 4;
//   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//   {
//     Error_Handler();
//   }

//   /** Initializes the CPU, AHB and APB buses clocks
//   */
//   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                               |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

//   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//   {
//     Error_Handler();
//   }
// }

// /* USER CODE BEGIN 4 */
// /*
//  * printf 重定向已移除 (USART2 给 VOFA 用)
//  * 测试期间如果想 printf 调试, 可以临时打开下面的 _write 实现
//  */
// /*
// #ifdef __GNUC__
// int _write(int fd, char *ptr, int len)
// {
//     HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
//     return len;
// }
// #endif
// */
// /* USER CODE END 4 */

// /**
//   * @brief  This function is executed in case of error occurrence.
//   * @retval None
//   */
// void Error_Handler(void)
// {
//   /* USER CODE BEGIN Error_Handler_Debug */
//   /* User can add his own implementation to report the HAL error return state */
//   __disable_irq();
//   while (1)
//   {
//   }
//   /* USER CODE END Error_Handler_Debug */
// }
// #ifdef USE_FULL_ASSERT
// /**
//   * @brief  Reports the name of the source file and the source line number
//   *         where the assert_param error has occurred.
//   * @param  file: pointer to the source file name
//   * @param  line: assert_param error line source number
//   * @retval None
//   */
// void assert_failed(uint8_t *file, uint32_t line)
// {
//   /* USER CODE BEGIN 6 */
//   /* User can add his own implementation to report the file name and line number,
//      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//   /* USER CODE END 6 */
// }
// #endif /* USE_FULL_ASSERT */