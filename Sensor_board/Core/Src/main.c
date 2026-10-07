/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_CH_COUNT                    14u

/*
 * Frozen/proven 16-bit SPI protocol.
 *
 * SPI data size:
 *   16-bit
 *
 * PING:
 *   main sends 0x5AA5
 *   sensor replies 0xA55A
 *
 * READ:
 *   main sends 0x00C8
 *   sensor replies 14 x uint16_t ADC values
 */
#define SENSOR_CMD_READ                 200u
#define SENSOR_CMD_PING                 0x5AA5u
#define SENSOR_ACK_PING                 0xA55Au
#define SENSOR_READ_MAGIC               0xBEEFu
#define SENSOR_READ_CHECK_SEED          0x1357u
#define SENSOR_READ_FRAME_WORDS         (ADC_CH_COUNT + 3u)
#define SENSOR_READ_VALUE_OFFSET        2u
#define SENSOR_READ_CHECK_INDEX         (SENSOR_READ_FRAME_WORDS - 1u)

/*
 * Power-on race protection.
 *
 * Both boards power on together. If this slave arms SPI too early, garbage
 * clocks or floating lines during main-board boot can leave the SPI peripheral
 * in a bad state. Waiting before the first arm reproduces the manual behavior
 * of pressing the sensor-board reset after the main board is already stable.
 */
#define SENSOR_STARTUP_ARM_DELAY_MS     700u
#define SENSOR_STARTUP_CS_HIGH_MS       50u
#define SENSOR_STARTUP_TIMEOUT_MS       3000u

/*
 * If receive is not armed and SPI is idle, retry arming periodically.
 */
#define SENSOR_REARM_PERIOD_MS          10u

/*
 * If a slave transmit is armed but the master never clocks the response, abort
 * and re-arm command receive. This prevents the slave from getting stuck in
 * BUSY_TX after a bad/partial transaction during power-on.
 */
#define SENSOR_TX_STUCK_MS              100u
#define SENSOR_CS_HIGH_TX_GRACE_MS      2u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define SENSOR_CS_Pin                   GPIO_PIN_2
#define SENSOR_CS_GPIO_Port             GPIOD
#define SENSOR_CS_IS_LOW()              (HAL_GPIO_ReadPin(SENSOR_CS_GPIO_Port, SENSOR_CS_Pin) == GPIO_PIN_RESET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile uint16_t adc_buf[ADC_CH_COUNT];
volatile uint16_t adc_shadow[ADC_CH_COUNT];

static volatile uint16_t rx_cmd = 0u;
static uint16_t tx_ack = SENSOR_ACK_PING;
static uint16_t tx_adc_frame[ADC_CH_COUNT];
static uint16_t tx_read_frame[SENSOR_READ_FRAME_WORDS];
static uint16_t sensor_read_sequence = 0u;

/*
 * SPI slave state/debug.
 * Keep these visible for Live Watch.
 */
volatile uint8_t sensor_rx_armed = 0u;
volatile uint8_t sensor_tx_busy = 0u;

volatile uint32_t sensor_last_arm_ms = 0u;
volatile uint32_t sensor_tx_start_ms = 0u;
volatile uint32_t sensor_last_cs_high_ms = 0u;

volatile uint16_t sensor_last_cmd = 0u;
volatile uint16_t sensor_last_ack = 0u;
volatile HAL_StatusTypeDef sensor_last_hal_status = HAL_OK;

volatile uint32_t sensor_arm_ok_count = 0u;
volatile uint32_t sensor_arm_fail_count = 0u;
volatile uint32_t sensor_ping_count = 0u;
volatile uint32_t sensor_read_count = 0u;
volatile uint32_t sensor_unknown_cmd_count = 0u;
volatile uint32_t sensor_tx_start_fail_count = 0u;
volatile uint32_t sensor_tx_done_count = 0u;
volatile uint32_t sensor_error_count = 0u;
volatile uint32_t sensor_recover_count = 0u;
volatile uint32_t sensor_cs_low_count = 0u;
volatile uint32_t sensor_cs_high_count = 0u;
volatile uint32_t sensor_cs_resync_count = 0u;
volatile uint8_t sensor_cs_level = 1u;
volatile uint8_t sensor_recover_pending = 0u;
volatile uint8_t sensor_startup_ready = 0u;
volatile uint8_t sensor_transaction_start_pending = 0u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void sensor_arm_receive(void);
static void sensor_abort_to_receive(void);
static void sensor_recover_spi(void);
static void sensor_service_spi(void);
static uint16_t sensor_read_checksum(const uint16_t *frame);
static void sensor_prepare_adc_frame(void);
static void sensor_wait_startup_cs_idle(void);
static uint8_t sensor_start_transmit(uint8_t *data, uint16_t frame_count);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void sensor_abort_to_receive(void)
{
    uint16_t i;

    sensor_cs_resync_count++;

    sensor_rx_armed = 0u;
    sensor_tx_busy = 0u;
    rx_cmd = 0u;

    for (i = 0u; i < ADC_CH_COUNT; i++)
    {
        tx_adc_frame[i] = 0u;
    }

    for (i = 0u; i < SENSOR_READ_FRAME_WORDS; i++)
    {
        tx_read_frame[i] = 0u;
    }

    (void)HAL_SPI_Abort(&hspi1);

    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        sensor_last_hal_status = HAL_SPI_DeInit(&hspi1);
        sensor_last_hal_status = HAL_SPI_Init(&hspi1);
    }

    if (SENSOR_CS_IS_LOW())
    {
        sensor_arm_receive();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != SENSOR_CS_Pin)
    {
        return;
    }

    if (SENSOR_CS_IS_LOW())
    {
        sensor_cs_level = 0u;
        sensor_cs_low_count++;

        if (sensor_startup_ready == 0u)
        {
            return;
        }

        sensor_transaction_start_pending = 1u;

        return;
    }

    sensor_cs_level = 1u;
    sensor_cs_high_count++;
    sensor_last_cs_high_ms = HAL_GetTick();

    if (sensor_startup_ready == 0u)
    {
        return;
    }
}

static void sensor_arm_receive(void)
{
    HAL_StatusTypeDef ret;

    if (!SENSOR_CS_IS_LOW())
    {
        return;
    }

    if (sensor_tx_busy != 0u)
    {
        return;
    }

    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        return;
    }

    rx_cmd = 0u;
    sensor_rx_armed = 0u;

    /*
     * SPI is 16-bit.
     * Size = 1 means receive one uint16_t command frame.
     */
    ret = HAL_SPI_Receive_IT(&hspi1,
                             (uint8_t *)&rx_cmd,
                             1u);

    sensor_last_hal_status = ret;
    sensor_last_arm_ms = HAL_GetTick();

    if (ret == HAL_OK)
    {
        sensor_rx_armed = 1u;
        sensor_arm_ok_count++;
    }
    else
    {
        sensor_arm_fail_count++;
    }
}

static uint8_t sensor_start_transmit(uint8_t *data, uint16_t frame_count)
{
    HAL_StatusTypeDef ret;

    if ((data == 0) || (frame_count == 0u))
    {
        sensor_arm_receive();
        return 0u;
    }

    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        sensor_arm_receive();
        return 0u;
    }

    /*
     * SPI is 16-bit.
     * frame_count is the number of uint16_t frames.
     */
    ret = HAL_SPI_Transmit_IT(&hspi1,
                              data,
                              frame_count);

    sensor_last_hal_status = ret;

    if (ret == HAL_OK)
    {
        sensor_rx_armed = 0u;
        sensor_tx_busy = 1u;
        sensor_cs_level = SENSOR_CS_IS_LOW() ? 0u : 1u;
        sensor_tx_start_ms = HAL_GetTick();
        return 1u;
    }

    sensor_tx_start_fail_count++;
    sensor_arm_receive();
    return 0u;
}

static void sensor_recover_spi(void)
{
    uint16_t i;

    sensor_recover_count++;
    sensor_recover_pending = 0u;

    sensor_rx_armed = 0u;
    sensor_tx_busy = 0u;
    rx_cmd = 0u;

    for (i = 0u; i < ADC_CH_COUNT; i++)
    {
        tx_adc_frame[i] = 0u;
    }

    for (i = 0u; i < SENSOR_READ_FRAME_WORDS; i++)
    {
        tx_read_frame[i] = 0u;
    }

    HAL_SPI_Abort(&hspi1);
    sensor_last_hal_status = HAL_SPI_DeInit(&hspi1);
    sensor_last_hal_status = HAL_SPI_Init(&hspi1);

    sensor_arm_receive();
}

static void sensor_service_spi(void)
{
    uint32_t now;

    now = HAL_GetTick();

    if (sensor_recover_pending != 0u)
    {
        sensor_recover_spi();
        return;
    }

    if (sensor_transaction_start_pending != 0u)
    {
        sensor_transaction_start_pending = 0u;

        if (SENSOR_CS_IS_LOW())
        {
            sensor_abort_to_receive();
            return;
        }
    }

    /*
     * CS-high means the master ended the transaction. If TX-complete has not
     * arrived shortly after that, the response was partial/stuck and should be
     * dropped before the next command.
     */
    if ((sensor_tx_busy != 0u) &&
        (!SENSOR_CS_IS_LOW()) &&
        ((now - sensor_last_cs_high_ms) >= SENSOR_CS_HIGH_TX_GRACE_MS))
    {
        sensor_abort_to_receive();
        return;
    }

    /*
     * Recover from a transmit response that was never clocked by the master.
     */
    if ((sensor_tx_busy != 0u) &&
        ((now - sensor_tx_start_ms) >= SENSOR_TX_STUCK_MS))
    {
        sensor_recover_spi();
        return;
    }

    /*
     * If our software state says idle, SPI should also be ready. If not, the
     * HAL/peripheral state is wedged, so recover instead of waiting forever.
     */
    if ((sensor_rx_armed == 0u) &&
        (sensor_tx_busy == 0u) &&
        (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY))
    {
        sensor_recover_spi();
        return;
    }

    /*
     * If idle and receive is not armed, keep trying to arm it.
     */
    if ((sensor_rx_armed == 0u) &&
        (sensor_tx_busy == 0u) &&
        SENSOR_CS_IS_LOW() &&
        (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY) &&
        ((now - sensor_last_arm_ms) >= SENSOR_REARM_PERIOD_MS))
    {
        sensor_arm_receive();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint16_t i;

    if (hadc != &hadc1)
    {
        return;
    }

    for (i = 0u; i < ADC_CH_COUNT; i++)
    {
        adc_shadow[i] = adc_buf[i];
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1)
    {
        return;
    }

    sensor_rx_armed = 0u;
    sensor_last_cmd = rx_cmd;

    if (rx_cmd == SENSOR_CMD_PING)
    {
        sensor_ping_count++;
        sensor_last_ack = tx_ack;

        (void)sensor_start_transmit((uint8_t *)&tx_ack, 1u);
        return;
    }

    if (rx_cmd == SENSOR_CMD_READ)
    {
        sensor_read_count++;
        sensor_prepare_adc_frame();

        /*
         * SPI is 16-bit.
         * READ frame = magic, sequence, 14 x ADC, checksum.
         */
        (void)sensor_start_transmit((uint8_t *)tx_read_frame,
                                    SENSOR_READ_FRAME_WORDS);
        return;
    }

    sensor_unknown_cmd_count++;
    sensor_arm_receive();
}

static uint16_t sensor_read_checksum(const uint16_t *frame)
{
    uint16_t i;
    uint16_t check;

    check = SENSOR_READ_CHECK_SEED ^ SENSOR_CMD_READ;

    for (i = 0u; i < SENSOR_READ_CHECK_INDEX; i++)
    {
        check ^= frame[i];
    }

    return check;
}

static void sensor_prepare_adc_frame(void)
{
    uint16_t i;

    /*
     * Copy the latest ADC snapshot into a stable SPI TX buffer. Do not transmit
     * directly from adc_shadow because the ADC DMA callback can update it while
     * a multi-word SPI response is being clocked by the master.
     */
    for (i = 0u; i < ADC_CH_COUNT; i++)
    {
        tx_adc_frame[i] = adc_shadow[i];
    }

    sensor_read_sequence++;

    tx_read_frame[0u] = SENSOR_READ_MAGIC;
    tx_read_frame[1u] = sensor_read_sequence;

    for (i = 0u; i < ADC_CH_COUNT; i++)
    {
        tx_read_frame[SENSOR_READ_VALUE_OFFSET + i] = tx_adc_frame[i];
    }

    tx_read_frame[SENSOR_READ_CHECK_INDEX] =
        sensor_read_checksum(tx_read_frame);
}

static void sensor_wait_startup_cs_idle(void)
{
    uint32_t start_ms;
    uint32_t high_start_ms;
    uint32_t now;

    start_ms = HAL_GetTick();
    high_start_ms = start_ms;

    while ((HAL_GetTick() - start_ms) < SENSOR_STARTUP_TIMEOUT_MS)
    {
        now = HAL_GetTick();

        if (SENSOR_CS_IS_LOW())
        {
            high_start_ms = now;
        }
        else if ((now - high_start_ms) >= SENSOR_STARTUP_CS_HIGH_MS)
        {
            return;
        }
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1)
    {
        return;
    }

    sensor_tx_busy = 0u;
    sensor_tx_done_count++;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hspi1)
    {
        return;
    }

    sensor_error_count++;
    sensor_recover_pending = 1u;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /*
   * STM32F1 safety when SPI1 is remapped to PB3/PB4/PB5:
   *   PB3/PB4 are JTAG pins by default.
   * Disable JTAG but keep SWD enabled.
   */
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  for (uint16_t i = 0u; i < ADC_CH_COUNT; i++)
  {
      adc_buf[i] = 0u;
      adc_shadow[i] = 0u;
      tx_adc_frame[i] = 0u;
  }

  for (uint16_t i = 0u; i < SENSOR_READ_FRAME_WORDS; i++)
  {
      tx_read_frame[i] = 0u;
  }

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_CH_COUNT);

  /*
   * Important startup guard:
   * Wait for the main-board side to become stable. With software NSS, SPI RX
   * is armed only on a CS falling edge, so boot-time SCK noise while CS is high
   * cannot clock a garbage command into the slave.
   */
  HAL_Delay(SENSOR_STARTUP_ARM_DELAY_MS);
  sensor_wait_startup_cs_idle();

  sensor_startup_ready = 1u;
  sensor_arm_receive();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    sensor_service_spi();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
