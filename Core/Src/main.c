/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum {
  IDLE = 0,
  ON,
  DIST_ALERT,
  CONFIG,
  ALARM
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define   ALERT_TIMEOUT       3000
#define   CLICK_MIN_SEC       50
#define   CLICK_MAX_SEC       1000
#define   UNDER_PRESS_SEC     2000
#define   INTERNAL_DIST_THR   20
#define   MOTOR_TOGGLE_TIME   1000 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
uint32_t sensor_channel[NUM_SENSORS] = {ADC_CHANNEL_0,
                                        ADC_CHANNEL_1,
                                        ADC_CHANNEL_2,
                                        ADC_CHANNEL_3,
                                        ADC_CHANNEL_4,
                                        ADC_CHANNEL_5,
                                        ADC_CHANNEL_6,
                                        ADC_CHANNEL_7,
                                        ADC_CHANNEL_17,
                                        ADC_CHANNEL_18};

volatile uint32_t sensor_sav[NUM_SENSORS];
volatile uint32_t sensor_val[NUM_SENSORS];
uint32_t tolerance = 400;

uint16_t state = IDLE, prev_state = IDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
uint32_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel);
uint32_t Get_SensorDist(volatile uint32_t values[NUM_SENSORS], uint32_t save[NUM_SENSORS]);
uint32_t Get_InternalDist(volatile uint32_t values[NUM_SENSORS]);
void Sensor_SaveProfile(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ADC_ChannelConfTypeDef gConfig = {0};
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
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
  HAL_Delay(5);
  Sensor_SaveProfile();

  uint32_t sensor_dist = 0;
  uint32_t sensor_int_dist = 0;
  uint32_t alarm_timestamp = HAL_GetTick();

  uint8_t click = 0;
  uint8_t under_pressure = 0;

  GPIO_PinState btn_state;
  uint8_t btn_start = 0;
  uint32_t btn_timestamp;
  
  uint8_t config_flag = 0;
  uint8_t blink = 0;

  uint8_t motor_state = 0, motor_transition = 0;
  uint32_t motor_timestamp;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_ADC_Start_IT(&hadc1);
    HAL_Delay(5); // Cambiar delay por flag

    if (Get_SensorDist(sensor_val, (uint32_t *)sensor_sav) > tolerance) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    }
    else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

    // Leer botones
    // Medir tiempo con HAL_GetTick(). Si el tiempo de presión está entre un rango acotado: click
    // Si el tiempo de presión supera una cantidad: presión extendida
    // Estando dentro de CONFIG, no salir hasta que se suelte el botón

    btn_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);

    if (btn_state == GPIO_PIN_RESET) {
      if (btn_start == 0) {
        btn_start = 1;
        btn_timestamp = HAL_GetTick();
      }

      if (HAL_GetTick() - btn_timestamp > UNDER_PRESS_SEC) {
        under_pressure = 1;
      }
    }
    else {
      if (btn_start == 1) {
        btn_start = 0;
        uint32_t time_pressed = HAL_GetTick() - btn_timestamp;
        if (time_pressed > CLICK_MIN_SEC && time_pressed < CLICK_MAX_SEC) { 
          click = 1;
        }

        under_pressure = 0;
      }
      else {
        click = 0;
      }
    }

    // FSM
    switch (state)
    {
    case IDLE:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      if (click) {

        prev_state = IDLE;
        state = ON;
      }
      else if (under_pressure) {
        prev_state = IDLE;
        state = CONFIG;
      }
      
      break;

    case ON:
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      
      sensor_dist = Get_SensorDist(sensor_val, (uint32_t *)sensor_sav);
      sensor_int_dist = Get_InternalDist(sensor_val);

      if (sensor_dist > tolerance && sensor_int_dist >= INTERNAL_DIST_THR) {
        prev_state = ON;
        alarm_timestamp = HAL_GetTick();
        state = DIST_ALERT;
      }

      if (click) {
        prev_state = ON;
        state = IDLE;
      }
      else if (under_pressure) {
        prev_state = ON;
        state = CONFIG;
      }
      break;

    case DIST_ALERT:
      if (HAL_GetTick() - alarm_timestamp > ALERT_TIMEOUT) {
        state = ALARM;
        prev_state = DIST_ALERT;
        motor_transition = 1;
      }

      sensor_dist = Get_SensorDist(sensor_val, (uint32_t *)sensor_sav);
      sensor_int_dist = Get_InternalDist(sensor_val);
      if (sensor_dist <= tolerance || sensor_int_dist < INTERNAL_DIST_THR) {
        state = ON;
        prev_state = DIST_ALERT;
      }

      if (click) {
        prev_state = DIST_ALERT;
        state = IDLE;
      }
      break;

    case CONFIG:
      if (!under_pressure) {
        state = prev_state;
        prev_state = CONFIG;
        config_flag = 0;
        blink = 0;
      }
      else {
        if (config_flag == 0) {          
          HAL_ADC_Start_IT(&hadc1);
          HAL_Delay(5);
          Sensor_SaveProfile();

          config_flag = 1;
        }
        else {
          if (!blink) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            HAL_Delay(100);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            blink = 1;
          }
        }
      }
      break;

    case ALARM:
      // ENCENDER
      if (motor_transition) {
        motor_transition = 0;
        motor_timestamp = HAL_GetTick();
      }

      if (HAL_GetTick() - motor_timestamp > MOTOR_TOGGLE_TIME) {
        if (motor_state) {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
          motor_transition = 1;
          motor_state = 0;
        }
        else {
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
          motor_transition = 1;
          motor_state = 1;
        }
      }

      sensor_dist = Get_SensorDist(sensor_val, (uint32_t *)sensor_sav);
      sensor_int_dist = Get_InternalDist(sensor_val);
      if (sensor_dist <= tolerance || sensor_int_dist < INTERNAL_DIST_THR) {
        state = ON;
        prev_state = ALARM;
        // Apagar motores
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        motor_state = 0;
        motor_transition = 0;
      }

      if (click) {
        motor_state = 0;
        motor_transition = 0;
        prev_state = ALARM;
        state = IDLE;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
      }
      break;
    }

    HAL_Delay(1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  gConfig = sConfig;
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint32_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel) {
  //gConfig.Channel = channel;
  //HAL_ADC_ConfigChannel(hadc, &gConfig);
  //HAL_ADC_Start(hadc);

  if (HAL_ADC_PollForConversion(hadc, 0) == HAL_OK) {
    return HAL_ADC_GetValue(hadc);
  }

  return 0;
}

uint32_t Get_SensorDist(volatile uint32_t values[NUM_SENSORS], uint32_t save[NUM_SENSORS]) {
  float dist = 0;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    dist += powf((float)values[i] - (float)save[i], 2);
  }

  return (uint32_t)sqrtf(dist);
}

uint32_t Get_InternalDist(volatile uint32_t values[NUM_SENSORS]) {
  float mean, sum = 0.0, std = 0.0;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sum += (float)values[i];
  }
  mean = sum / NUM_SENSORS;

  sum = 0.0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sum += powf((float)values[i] - mean, 2);
  }
  std = sqrtf(sum);

  return (uint32_t)std;
}

void Sensor_SaveProfile() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensor_sav[i] = sensor_val[i];
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
