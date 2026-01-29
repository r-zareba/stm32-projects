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
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
// #define DUAL_CORE_BOOT_SYNC_SEQUENCE  // DISABLED - M4 not used

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define BUFFER_SIZE                                                            \
  2048 // 200ms at 10.24kHz (IEC 61000-4-7 perfect FFT window: 2^11)
#define HALF_BUFFER_SIZE                                                       \
  1024 // 100ms per packet (power-of-2 for efficient processing)

/**
 * UART Packet Structure (4010 bytes total)
 *
 * All fields are uint16_t for natural 2-byte alignment.
 * NO __attribute__((packed)) needed - compiler adds no padding because
 * all fields are same size (uint16_t = 2 bytes).
 *
 * Memory layout is naturally compact:
 * - Each uint16_t at even address (0, 2, 4, 6, ...)
 * - No gaps between fields
 * - Total size: 2+2+2+2000+2000+2+2 = 4010 bytes
 */
typedef struct {
  uint16_t start_marker;                   // 0xFFFF
  uint16_t sequence;                       // Packet counter (0-65535, wraps)
  uint16_t count;                          // Samples per channel (always 1000)
  uint16_t vref_mv;                        // VDDA voltage in millivolts (from VREFINT)
  uint16_t voltage_data[HALF_BUFFER_SIZE]; // ADC1 voltage samples
  uint16_t current_data[HALF_BUFFER_SIZE]; // ADC2 current samples
  uint16_t checksum;                       // CRC16-MODBUS
  uint16_t end_marker;                     // 0xFFFE
} PacketData;

// DMA buffers in D2 SRAM (0x30000000) - NOT cacheable, no MPU needed!
uint32_t adc_buffer[BUFFER_SIZE] __attribute__((section(".dma_buffer")))
__attribute__((aligned(32)));

static PacketData tx_packet __attribute__((section(".dma_buffer")))
__attribute__((aligned(32)));

volatile uint8_t buffer_half_ready = 0;
volatile uint8_t uart_tx_busy = 0;

static uint16_t packet_sequence = 0;

// VREFINT calibration - addresses defined in stm32h7xx_ll_adc.h
static volatile uint16_t vdda_mv = 3300;  // Default to 3.3V, updated by VREFINT reading
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void transmit_buffer_uart(uint32_t *data, uint16_t size) {
  if (uart_tx_busy) {
    return;
  }

  tx_packet.start_marker = 0xFFFF;
  tx_packet.sequence = packet_sequence++;
  tx_packet.count = size;
  tx_packet.vref_mv = vdda_mv;  // Include calibrated VDDA voltage

  for (uint16_t i = 0; i < size; i++) {
    tx_packet.voltage_data[i] = (uint16_t)(data[i] & 0xFFFF);
    tx_packet.current_data[i] = (uint16_t)((data[i] >> 16) & 0xFFFF);
  }

  uint8_t *crc_start = (uint8_t *)&tx_packet.sequence;
  uint16_t crc_length = sizeof(tx_packet.sequence) + sizeof(tx_packet.count) +
                        sizeof(tx_packet.vref_mv) +
                        sizeof(tx_packet.voltage_data) +
                        sizeof(tx_packet.current_data);
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < crc_length; i++) {
    crc ^= crc_start[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }

  tx_packet.checksum = crc;
  tx_packet.end_marker = 0xFFFE;

  uart_tx_busy = 1;
  HAL_StatusTypeDef status =
      HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&tx_packet, sizeof(tx_packet));

  if (status != HAL_OK) {
    uart_tx_busy = 0;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Red LED on error
  } else {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Toggle Green LED
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0))
    ;
  if (timeout < 0) {
    Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
       /* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* When system initialization is finished, Cortex-M7 will release Cortex-M4 by
  means of HSEM notification */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /*Take HSEM */
  HAL_HSEM_FastTake(HSEM_ID_0);
  /*Release HSEM in order to notify the CPU2(CM4)*/
  HAL_HSEM_Release(HSEM_ID_0, 0);
  /* wait until CPU2 wakes up from stop mode */
  timeout = 0xFFFF;
  while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0))
    ;
  if (timeout < 0) {
    Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
       /* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  // Calibrate ADCs (H7 needs calibration like L4)
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  // Clear buffer before starting to verify ADC is actually writing new data
  for (int i = 0; i < BUFFER_SIZE; i++) {
    adc_buffer[i] = 0xDEADBEEF; // Known pattern
  }

  // Start TIM6 to trigger ADC conversions at 10 kHz
  HAL_TIM_Base_Start(&htim6);

  if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_buffer,
                                   BUFFER_SIZE) != HAL_OK) {
    Error_Handler();
  }

  // Debug: Blink green LED 3 times to confirm initialization
  for (int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(200);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_tx_time = 0;
    static uint32_t last_vrefint_time = 0;
    uint32_t current_time = HAL_GetTick();

    // Read VREFINT once per second for VDDA calibration
    if (current_time - last_vrefint_time >= 1000) {
      last_vrefint_time = current_time;

      // Start ADC3 conversion for VREFINT
      HAL_ADC_Start(&hadc3);
      if (HAL_ADC_PollForConversion(&hadc3, 100) == HAL_OK) {
        uint32_t vrefint_raw = HAL_ADC_GetValue(&hadc3);

        // Calculate VDDA from VREFINT reading
        // VDDA = (VREFINT_CAL_VREF * VREFINT_CAL) / vrefint_raw
        uint16_t vrefint_cal = *VREFINT_CAL_ADDR;
        if (vrefint_raw > 0) {
          vdda_mv = (uint16_t)((VREFINT_CAL_VREF * vrefint_cal) / vrefint_raw);
        }
      }
      HAL_ADC_Stop(&hadc3);
    }

    if (uart_tx_busy && huart3.gState == HAL_UART_STATE_READY) {
      uart_tx_busy = 0;
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // Toggle Green LED
    }

    if (uart_tx_busy && (current_time - last_tx_time > 150)) {
      uart_tx_busy = 0;
      HAL_UART_Abort(&huart3);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Red LED on timeout
    }

    if (buffer_half_ready == 1 && !uart_tx_busy) {
      transmit_buffer_uart(&adc_buffer[0], HALF_BUFFER_SIZE);
      buffer_half_ready = 0;
      last_tx_time = current_time;
    } else if (buffer_half_ready == 2 && !uart_tx_busy) {
      transmit_buffer_uart(&adc_buffer[HALF_BUFFER_SIZE], HALF_BUFFER_SIZE);
      buffer_half_ready = 0;
      last_tx_time = current_time;
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
   */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
   */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    buffer_half_ready = 1;
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // Toggle Yellow LED on half-complete
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    buffer_half_ready = 2;
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // Toggle Yellow LED on complete
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    uart_tx_busy = 0;
  }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    HAL_Delay(10);
    HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_buffer, BUFFER_SIZE);
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
