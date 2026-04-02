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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//=========MOTO DEFINITION=========//
#define MM_PER_DEGREE 0.111111f

#define MOTOR_PIN GPIO_PIN_12
#define MOTOR_PORT GPIOB
#define MOTOR_PIN2 GPIO_PIN_13
#define MOTOR_PORT2 GPIOB

//Solenoid GPIO for Actuator
#define SOLENOID_PIN GPIO_PIN_14
#define SOLENOID_PORT GPIOB
#define SOLENOID_PIN2 GPIO_PIN_15
#define SOLENOID_PORT2 GPIOB

#define WHITE_KEY 0
#define BLACK_KEY 1
#define SHORT_PRESS 1
#define LONG_PRESS 5


//LED for debugging
#define LED_PIN GPIO_PIN_13
#define LED_PORT GPIOC

/* ==================== PID Gains ==================== */
/* Start with these, tune on real hardware:             */
/*   Kp: increase until fast response without overshoot */
/*   Ki: increase to eliminate steady‑state error       */
/*   Kd: increase to dampen oscillation                 */
#define PID_KP               0.1f //0.035f
#define PID_KI               0.0f //0.005f //0.001f //oscillation around 0.04 or 0.05
#define PID_KD               0.0f //0.00075f//0.06f

/* ==================== PID Timing ==================== */
/* TIM2 update fires at ~10 kHz.  We only run PID every */
/* PID_DIVIDER ticks → PID freq ≈ 1 kHz.                */
#define PID_DIVIDER          10
#define PID_DT               (1.0f / 1000.0f)   /* 1 ms */

/* Integral anti‑windup clamp                           */
#define INTEGRAL_MAX         5000.0f

/* Dead‑band: if |error| <= this, stop the motor        */
#define DEADBAND             5.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//angle tracking variables
volatile float total_angle = 0;
float previous_angle = 0;
float initial_angle = 0.0f;

volatile float target_position = 0.0f;   // mm
volatile uint8_t uart_ready = 0;
char uart_msg[100];

volatile uint32_t adc_value;
volatile float debug_angle = 0;
volatile float debug_dist = 0;
volatile uint8_t target_reached_flag = 0;


//pid variables
float pid_error = 0;
float pid_integral = 0;
float pid_derivative = 0;
float pid_prev_error = 0;
float pid_output = 0;

uint32_t hold_start_time = 0;
volatile uint8_t holding = 0;
volatile float current_position = 0.0f;
uint32_t last_pid_time = 0;

float targets[] = {50.0f, 100.0f, 0.0f, 150.0f,175.0f,-100.0f, 130.0f};  // Example target positions in mm
volatile int current_target_index = 0;

volatile uint8_t solenoid_trigger = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

float get_angle(void);
void Motor_SetOutput(float output);
void Update_Position(void);
void press(int finger, int duration);
float PID_Compute(volatile float current_position);
//void Run_PID(void);
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
//am i still here
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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_value, 1);
 
HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);  // Ensure LED is off at start - ACTIVE LOW
//Start PWM channels for motor control
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

// Read initial angle so we start from 0

previous_angle = get_angle();
initial_angle = get_angle();
total_angle = 0.0f;
//HAL_UART_Transmit(&huart1, (uint8_t*)"Start\r\n", 7, 100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // --- UART debug ---
    if (uart_ready)
    {
        uart_ready = 0;

        sprintf(uart_msg, "Angle: %.2f | Dist: %.2f\r\n",
                debug_angle, debug_dist);

        HAL_UART_Transmit(&huart1, (uint8_t*)uart_msg, strlen(uart_msg), 100);
    }

    // --- Handle target reached event (from ISR) ---
    if (target_reached_flag)
    {
        target_reached_flag = 0;

        // Activate solenoid + LED
        solenoid_trigger = 1;

        HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

        hold_start_time = HAL_GetTick();
    }

    // --- Holding logic (timed in main loop) ---
    if (holding)
    {
        if (HAL_GetTick() - hold_start_time > 500)
        {
            holding = 0;

            // Deactivate solenoid + LED
            solenoid_trigger = 0;

            HAL_GPIO_WritePin(SOLENOID_PORT, SOLENOID_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

            // Move to next target
            current_target_index++;

            if (current_target_index >= 7)
                current_target_index = 0;

            target_position = targets[current_target_index];
        }
    }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 159;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


float get_angle(void)
{
    float voltage = (adc_value / 4095.0f) * 3.3f;

    // your conversion logic here
    float angle = (voltage / 3.3f) * 360.0f;

    return angle;
}
void Motor_SetOutput(float output)
{
    // Clamp to safe range (-1 to 1)
    if (output > 1.0f) output = 1.0f;
    if (output < -1.0f) output = -1.0f;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t pwm = (uint32_t)(fabsf(output) * arr);

    if (output > 0)
    {
        // Forward
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm); // PB0 (IN1)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);   // PB1 (IN2)
    }
    else if (output < 0)
    {
        // Reverse
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);   // PB0 (IN1)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm); // PB1 (IN2)
    }
    else
    {
        // Stop
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}
float PID_Compute(volatile float current_position)
{
    float error = target_position - current_position;

    // Integral
    pid_integral += error * PID_DT;

    if (pid_integral > INTEGRAL_MAX) pid_integral = INTEGRAL_MAX;
    if (pid_integral < -INTEGRAL_MAX) pid_integral = -INTEGRAL_MAX;

    // Derivative
    static float filtered_derivative = 0;
float raw_derivative = (error - pid_prev_error) / PID_DT;

// Low-pass filter (alpha = 0.1–0.2)
filtered_derivative = 0.1f * raw_derivative + 0.9f * filtered_derivative;

float derivative = filtered_derivative;

    // PID output
    float output = PID_KP * error
                 + PID_KI * pid_integral
                 + PID_KD * derivative;

    pid_prev_error = error;

    return output;
}
void Update_Position(void){
    float current_angle = get_angle();

    float position = current_angle - initial_angle;

    //unwrap so it handles passing through 0/360 boundary
    if (position > 180.0f)  position -= 360.0f;
    if (position < -180.0f) position += 360.0f;

    total_angle = position;  // assign, don't accumulate

}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    

        // --- Position update ---
        Update_Position();
        current_position = total_angle * MM_PER_DEGREE;

        debug_angle = total_angle;
        debug_dist = current_position;

        uart_ready = 1;

        // --- PID control ---
        if (!holding)
        {
            float error = target_position - current_position;

            if (fabsf(error) < DEADBAND)
            {
                Motor_SetOutput(0);
                pid_integral = 0;

                holding = 1;

                // Signal main loop only
                target_reached_flag = 1;
            }
            else
            {
                float output = PID_Compute(current_position);
                Motor_SetOutput(output);
            }
        }
        else
        {
            Motor_SetOutput(0);
        }
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
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    HAL_Delay(500);
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
