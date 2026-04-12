/*
 * main_user_code.c
 *
 * ====== 本文件不是独立源文件! ======
 * 本文件包含需要插入到 CubeMX 自动生成的 main.c 中的用户代码片段
 * 请按照注释中的标记位置, 将代码复制到 main.c 的对应 USER CODE 区域
 *
 * CubeMX 生成的 main.c 中有如下标记:
 *   /* USER CODE BEGIN Includes * /
 *   /* USER CODE END Includes * /
 *   /* USER CODE BEGIN 2 * /       <- 初始化代码
 *   /* USER CODE END 2 * /
 *   /* USER CODE BEGIN 3 * /       <- 主循环代码
 *   /* USER CODE END 3 * /
 */


/* ======================================================================
 * [1] 插入到 USER CODE BEGIN Includes
 * ====================================================================== */
// #include "app_common.h"


/* ======================================================================
 * [2] 插入到 USER CODE BEGIN 2 (初始化, 在 MX_xxx_Init() 之后)
 *
 * 初始化顺序很重要, 必须按以下顺序执行:
 *    1) 按键/LED 先初始化 (状态清零)
 *    2) 编码器启动计数
 *    3) 电机 PWM 启动 (初始占空比=0)
 *    4) 陀螺仪初始化 + 零漂校准 (~2秒, 阻塞)
 *    5) 风扇启动 (如需要, ~24秒渐加速, 阻塞)
 *    6) 传感器板初始化
 *    7) 最后启动 TIM7 定时中断 (1ms 控制循环开始)
 * ====================================================================== */
/*
  app_button_init();
  app_encoder_init();
  app_motor_init();
  app_gyro_init();
  app_fengshan(1);
  app_adc_init();
  HAL_TIM_Base_Start_IT(&htim7);
*/


/* ======================================================================
 * [3] 插入到 USER CODE BEGIN 3 (主循环 while(1) 内部)
 *
 * 主循环仅用于调试打印, 所有控制逻辑在 TIM7 中断中执行
 * ====================================================================== */
/*
    printf("%f,", jiaosudutobe);
    printf("%f\r\n", jiaosudu);
    HAL_Delay(10);
*/


/* ======================================================================
 * [4] printf 重定向 - 插入到 USER CODE BEGIN 4 或单独文件
 *
 * 将 printf 输出到 USART2 (PD5=TX, PD6=RX)
 * GCC: 覆写 _write()
 * Keil: 启用 MicroLIB + 覆写 fputc()
 * ====================================================================== */
/*
#ifdef __GNUC__
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
#else
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
#endif
*/
