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
#include "cmsis_os.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>  // For sprintf
#include <string.h> // For strlen
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLAME_SENSOR_PIN GPIO_PIN_0
#define FLAME_SENSOR_PORT GPIOA
#define LED_PIN GPIO_PIN_5
#define LED_PORT GPIOA
#define RGB_RED_PIN    GPIO_PIN_6
#define RGB_GREEN_PIN  GPIO_PIN_7
#define RGB_BLUE_PIN   GPIO_PIN_0
#define RGB_RED_PORT   GPIOA
#define RGB_GREEN_PORT GPIOA
#define RGB_BLUE_PORT  GPIOB
#define FLAME_THRESHOLD 2000  // Adjust based on your sensor
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t xFlameTaskHandle = NULL;
TaskHandle_t xAlertTaskHandle = NULL;
TaskHandle_t xSerialTaskHandle = NULL;
QueueHandle_t xFlameQueue = NULL;
uint16_t adc_value = 0; // To store ADC reading
uint8_t flame_status = 0; // To store flame status (1 = detected, 0 = not detected)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void vFlameSensorTask(void *pvParameters);
void vAlertTask(void *pvParameters);
void vSerialTask(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // Ensure initial states
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
  // Initialize RGB LED to off
  HAL_GPIO_WritePin(RGB_RED_PORT, RGB_RED_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RGB_GREEN_PORT, RGB_GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RGB_BLUE_PORT, RGB_BLUE_PIN, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
}

/* USER CODE BEGIN 4 */
// Flame Sensor Task
void vFlameSensorTask(void *pvParameters) {
    for (;;) {
        // Start ADC conversion
        HAL_ADC_Start(&hadc1);
        // Wait for conversion to complete
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
            // Read ADC value
            adc_value = HAL_ADC_GetValue(&hadc1);
            // Check if flame is detected
            flame_status = (adc_value > FLAME_THRESHOLD) ? 1 : 0;
            // Send status to queue
            xQueueOverwrite(xFlameQueue, &flame_status);
        }
        // Delay for 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Alert Task
void vAlertTask(void *pvParameters) {
  uint8_t flame_status;

  for(;;) {
    xQueuePeek(xFlameQueue, &flame_status, portMAX_DELAY);

    if(flame_status) {
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
      // Set RGB to Red
      HAL_GPIO_WritePin(RGB_RED_PORT, RGB_RED_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RGB_GREEN_PORT, RGB_GREEN_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RGB_BLUE_PORT, RGB_BLUE_PIN, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
      // Set RGB to Green
      HAL_GPIO_WritePin(RGB_RED_PORT, RGB_RED_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RGB_GREEN_PORT, RGB_GREEN_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(RGB_BLUE_PORT, RGB_BLUE_PIN, GPIO_PIN_RESET);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Serial Task
void vSerialTask(void *pvParameters) {
  uint8_t flame_status;
  char msg[20];

  for(;;) {
    xQueuePeek(xFlameQueue, &flame_status, portMAX_DELAY);

    if(flame_status) {
      sprintf(msg, "FLAME DETECTED!\r\n");
    } else {
      sprintf(msg, "System Normal\r\n");
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(500));
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
