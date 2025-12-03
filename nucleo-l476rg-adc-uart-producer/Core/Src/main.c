/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> // For memcpy() in packet transmission
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// DMA Buffer for ADC sampling (circular buffer)
#define BUFFER_SIZE 2000      // Total buffer: 2000 samples (200ms at 10kHz)
#define HALF_BUFFER_SIZE 1000 // Half buffer: 1000 samples (100ms at 10kHz)

uint16_t adc_buffer[BUFFER_SIZE]; // Circular buffer filled by DMA
volatile uint8_t buffer_half_ready =
    0; // 0=none, 1=first half ready, 2=second half ready
volatile uint8_t uart_tx_busy = 0; // 0=idle, 1=transmission in progress

// Packet structure for UART transmission
typedef struct {
  uint16_t start_marker;               // 0xAA55 - synchronization marker
  uint16_t sequence_number;            // Packet counter (0-65535, wraps around)
  uint16_t sample_count;               // Number of ADC samples in this packet
  uint16_t adc_data[HALF_BUFFER_SIZE]; // ADC samples (1000 values)
  uint16_t checksum;                   // CRC16 for integrity validation
  uint16_t end_marker;                 // 0x55AA - end of packet marker
} __attribute__((packed)) ADCPacket;

// Global variables for packet transmission
static uint16_t packet_sequence = 0; // Increments with each packet sent
static ADCPacket tx_packet; // Packet buffer (static to avoid stack overflow)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Calculate CRC16 checksum (MODBUS variant)
 * @param  data: Pointer to data buffer (uint16_t array)
 * @param  count: Number of uint16_t elements to process
 * @retval 16-bit CRC value
 */
uint16_t calculate_crc16(uint16_t *data, uint16_t count) {
  uint16_t crc = 0xFFFF;            // Initial value
  uint8_t *bytes = (uint8_t *)data; // Process as bytes
  uint16_t byte_count = count * 2;  // Convert word count to byte count

  for (uint16_t i = 0; i < byte_count; i++) {
    crc ^= bytes[i]; // XOR byte into CRC

    for (uint8_t j = 0; j < 8; j++) { // Process each bit
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001; // Apply polynomial if LSB is 1
      } else {
        crc = crc >> 1; // Just shift if LSB is 0
      }
    }
  }

  return crc;
}

/**
 * @brief  Build packet and transmit ADC buffer via UART with DMA
 * @param  data: Pointer to ADC data buffer
 * @param  size: Number of samples to transmit
 * @retval None
 */
void transmit_buffer_uart(uint16_t *data, uint16_t size) {
  if (uart_tx_busy) {
    // UART transmission already in progress - skip this buffer
    // In production, might want to log this as a data loss event
    return;
  }

  // Build packet header
  tx_packet.start_marker = 0xAA55; // Sync pattern for packet detection
  tx_packet.sequence_number = packet_sequence++; // Increment packet counter
  tx_packet.sample_count = size;                 // Number of samples

  // Copy ADC data from DMA buffer to transmit packet
  memcpy(tx_packet.adc_data, data, size * sizeof(uint16_t));

  // Calculate CRC16 checksum over: sequence_number + sample_count + adc_data
  // Use byte pointer to avoid alignment warning with packed struct
  uint8_t *checksum_data_bytes = (uint8_t *)&tx_packet.sequence_number;
  uint16_t checksum_byte_count =
      (1 + 1 + size) * 2; // (seq + count + samples) * 2 bytes per word

  // Calculate CRC on bytes, then convert back to uint16_t format for function
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < checksum_byte_count; i++) {
    crc ^= checksum_data_bytes[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  tx_packet.checksum = crc;

  // Add end marker
  tx_packet.end_marker = 0x55AA; // Different from start for validation

  // Calculate total packet size in bytes
  // Header: start(2) + seq(2) + count(2) = 6 bytes
  // Data: size * 2 bytes
  // Trailer: checksum(2) + end(2) = 4 bytes
  uint16_t packet_size = 6 + (size * 2) + 4;

  // Start non-blocking UART transmission via DMA
  uart_tx_busy = 1; // Set flag before starting transmission
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&tx_packet, packet_size);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // Calibrate ADC for accurate measurements (required before first use)
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // Start ADC with DMA in circular mode
  // Buffer will be filled continuously: samples 0-999, then 1000-1999, then
  // wrap to 0-999, etc. Callbacks fire at half-complete (999) and complete
  // (1999)
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, BUFFER_SIZE);

  // Start timer to trigger ADC conversions at 10kHz
  // Timer MUST be started AFTER ADC is ready
  HAL_TIM_Base_Start(&htim6);

  // DEBUG: Send test packet immediately to verify UART is working
  // Fill buffer with test pattern
  for (int i = 0; i < HALF_BUFFER_SIZE; i++) {
    adc_buffer[i] = i % 256; // Pattern: 0,1,2...255,0,1,2...
  }
  // Force transmission (bypassing normal flow)
  transmit_buffer_uart(&adc_buffer[0], HALF_BUFFER_SIZE);
  HAL_Delay(100); // Wait for transmission

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // DEBUG: Blink LED to show main loop is running
    static uint32_t last_blink = 0;
    if (HAL_GetTick() - last_blink > 500) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      last_blink = HAL_GetTick();
    }

    // DEBUG: Monitor callback flags
    static uint8_t last_half_ready = 0;
    if (buffer_half_ready != last_half_ready) {
      // Flag changed - toggle LED rapidly to show callbacks are firing
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(50);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      last_half_ready = buffer_half_ready;
    }

    // Check if first half of buffer is ready and UART is available
    if (buffer_half_ready == 1 && !uart_tx_busy) {
      // Transmit first half (samples 0-999)
      // DMA is currently filling second half (samples 1000-1999)
      transmit_buffer_uart(&adc_buffer[0], HALF_BUFFER_SIZE);
      buffer_half_ready = 0; // Clear flag after starting transmission
    }
    // Check if second half of buffer is ready and UART is available
    else if (buffer_half_ready == 2 && !uart_tx_busy) {
      // Transmit second half (samples 1000-1999)
      // DMA has wrapped around and is filling first half (samples 0-999)
      transmit_buffer_uart(&adc_buffer[HALF_BUFFER_SIZE], HALF_BUFFER_SIZE);
      buffer_half_ready = 0; // Clear flag after starting transmission
    }

    // Optional: Add low-power mode here if needed
    // __WFI();  // Wait For Interrupt - sleeps until next interrupt
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  ADC conversion half complete callback (DMA filled first half of
 * buffer)
 * @param  hadc: ADC handle
 * @retval None
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    // First half of buffer (samples 0-999) is now full and ready to transmit
    // DMA is currently filling the second half (samples 1000-1999)
    buffer_half_ready = 1;
  }
}

/**
 * @brief  ADC conversion complete callback (DMA filled second half of buffer)
 * @param  hadc: ADC handle
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    // Second half of buffer (samples 1000-1999) is now full and ready to
    // transmit DMA will wrap around and start filling first half again
    // (circular mode)
    buffer_half_ready = 2;
  }
}

/**
 * @brief  UART transmission complete callback (DMA finished sending packet)
 * @param  huart: UART handle
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    // UART DMA transmission complete - ready for next packet
    uart_tx_busy = 0;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
