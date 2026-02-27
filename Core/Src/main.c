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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ==================== Encoder ==================== */
#define ENCODER_PPR          334       /* Pulses per revolution (334PPR encoder)       */
#define ENCODER_CPR          (ENCODER_PPR * 4)  /* Counts per rev in TI12 mode = 1336  */
#define ENCODER_TIMER_PERIOD 65535     /* TIM3 ARR (16-bit)                            */

/* ==================== PWM ====================    */
#define PWM_MAX              99        /* TIM2 ARR = 99 → duty 0..99                   */
#define PWM_MIN              0

/* ==================== PID Gains ==================== */
/* Start with these, tune on real hardware:             */
/*   Kp: increase until fast response without overshoot */
/*   Ki: increase to eliminate steady‑state error       */
/*   Kd: increase to dampen oscillation                 */
#define PID_KP               5.0f
#define PID_KI               0.01f
#define PID_KD               0.1f

/* ==================== PID Timing ==================== */
/* TIM2 update fires at ~10 kHz.  We only run PID every */
/* PID_DIVIDER ticks → PID freq ≈ 1 kHz.                */
#define PID_DIVIDER          10
#define PID_DT               (1.0f / 1000.0f)   /* 1 ms */

/* Integral anti‑windup clamp                           */
#define INTEGRAL_MAX         5000.0f

/* Dead‑band: if |error| <= this, stop the motor        */
#define DEADBAND             2

/* ==================== Data Recording ==================== */
/* Record target & actual position into RAM arrays.         */
/* 2000 samples @ 1kHz = 2 seconds of data.                 */
#define RECORD_SIZE              2000
#define RECORD_TRIGGER_THRESHOLD 10    /* Start recording when |error| > this */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ---- PID state ---- */
typedef struct {
    float kp;
    float ki;
    float kd;

    int32_t target_position;   /* Target encoder count                     */
    int32_t current_position;  /* Current 32‑bit encoder count             */

    float   error;             /* Current error                            */
    float   prev_error;        /* Previous error (for D term)              */
    float   integral;          /* Accumulated integral                     */
    float   derivative;        /* Derivative term                          */
    float   output;            /* PID output (signed, ‑PWM_MAX..+PWM_MAX) */
} PID_t;

PID_t pid = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .target_position  = 0,
    .current_position = 0,
    .error      = 0.0f,
    .prev_error = 0.0f,
    .integral   = 0.0f,
    .derivative = 0.0f,
    .output     = 0.0f,
};

/* ---- Encoder overflow tracking ---- */
int16_t  encoder_last_count = 0;   /* Previous raw TIM3 count (signed 16‑bit)  */
int32_t  encoder_total      = 0;   /* Accumulated 32‑bit position in counts    */

/* ---- ISR tick divider ---- */
volatile uint16_t pid_tick_counter = 0;

/* ---- Data recording buffers ---- */
volatile int32_t  rec_target[RECORD_SIZE];   /* Recorded target positions  */
volatile int32_t  rec_actual[RECORD_SIZE];   /* Recorded actual positions  */
volatile uint16_t rec_index = 0;             /* Current write index        */
volatile uint8_t  rec_active = 0;            /* 1 = recording in progress  */
volatile uint8_t  rec_done   = 0;            /* 1 = recording finished     */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---------- Forward declarations ---------- */
void     PID_Compute(void);
void     Motor_SetOutput(float output);
int32_t  Encoder_GetPosition(void);

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* ==========================================================
   *  TEST MODE SWITCH
   *  Set to 1 = encoder-only test (motor off, safe for wiring check)
   *  Set to 0 = full PID demo with homing
   * ========================================================== */
  #define ENCODER_TEST_MODE  0

  /* ---- Start Encoder (TIM3) ---- */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0);

#if !ENCODER_TEST_MODE

  /* ---- Wake DRV8838 ---- */
  HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_SET);

  /* ---- Start PWM with 0% duty ---- */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* ============================================================
   *  HOMING SEQUENCE
   *  Motor slowly reverses until KY-003 Hall sensor detects magnet.
   *  KY-003: output = LOW when magnet is near (active-low).
   *
   *  In the real project, the slider moves backward along the rail
   *  until it hits the home position (magnet at start of rail).
   *  Then encoder is reset to 0 — this becomes the origin.
   * ============================================================ */
  #define HOMING_PWM_DUTY  30   /* Slow speed for homing (0–99) */

  /* Set direction = reverse (toward home) */
  HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_SET);   /* Reverse */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, HOMING_PWM_DUTY);

  /* Wait until Hall sensor triggers (LOW = magnet detected) */
  while (HAL_GPIO_ReadPin(HALL_SENSOR_GPIO_Port, HALL_SENSOR_Pin) == GPIO_PIN_SET)
  {
      /* Still moving toward home... */
      HAL_Delay(1);
  }

  /* ---- Magnet detected! Stop motor ---- */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);   /* PWM = 0 → stop */

  /* ---- Reset encoder to 0 — this is now our origin ---- */
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  encoder_last_count = 0;
  encoder_total      = 0;

  /* ---- Reset PID state ---- */
  pid.target_position  = 0;
  pid.current_position = 0;
  pid.error      = 0.0f;
  pid.prev_error = 0.0f;
  pid.integral   = 0.0f;
  pid.derivative = 0.0f;
  pid.output     = 0.0f;

  /* Small delay after homing */
  HAL_Delay(500);

  /* ---- Now start PID interrupt (closed-loop control begins) ---- */
  HAL_TIM_Base_Start_IT(&htim2);

#else
  /* ---- Encoder test: keep DRV8838 sleeping (motor off) ---- */
  HAL_GPIO_WritePin(SLEEP_GPIO_Port, SLEEP_Pin, GPIO_PIN_RESET);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* ----- Debug variables: watch these in CLion ----- */
  volatile int32_t  debug_position = 0;   /* 32-bit accumulated position   */
  volatile uint16_t debug_raw      = 0;   /* Raw TIM3 counter (0~65535)    */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if ENCODER_TEST_MODE
    debug_position = Encoder_GetPosition();
    debug_raw      = __HAL_TIM_GET_COUNTER(&htim3);
    HAL_Delay(100);

#else
    /* ====== HOLD AT HOME POSITION (0) ======
     * PID is running in TIM2 interrupt, holding position at 0.
     * Hand-turn the motor shaft → PID will fight back and
     * return to 0, demonstrating closed-loop disturbance rejection.
     *
     * Uncomment the demo steps below if you want to show
     * multi-position movement before holding.
     */

    /*  --- Optional demo steps (uncomment to use) ---
    pid.target_position = ENCODER_CPR;
    HAL_Delay(3000);

    pid.target_position = 0;
    HAL_Delay(3000);

    pid.target_position = ENCODER_CPR * 2;
    HAL_Delay(3000);

    pid.target_position = ENCODER_CPR / 2;
    HAL_Delay(3000);
    */

    /* Hold at 0 forever — PID maintains position in background */
    pid.target_position = 0;
    while (1) { }

#endif

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
}

/* USER CODE BEGIN 4 */

/**
 * @brief  Read 32-bit encoder position with 16-bit overflow tracking.
 *
 * TIM3 is a 16-bit counter (0‥65535).  When the motor spins continuously
 * the counter wraps around.  We detect the wrap by comparing the current
 * count to the previous count as a *signed* 16-bit delta.  This works
 * as long as the motor doesn't move more than 32767 counts between two
 * consecutive reads — at 1 kHz PID rate that allows up to ~24 revolutions
 * per second, well above the RS‑365PW's ~5 rev/s loaded speed.
 *
 * @retval 32-bit signed position in encoder counts.
 */
int32_t Encoder_GetPosition(void)
{
    int16_t current_raw = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t delta       = current_raw - encoder_last_count;
    encoder_last_count  = current_raw;

    /* If motor forward makes encoder count negative, negate delta.
     * Set ENCODER_DIR to -1 to invert, or +1 for normal.
     * If motor still runs away in one direction, flip this sign. */
    #define ENCODER_DIR  (-1)
    encoder_total += (int32_t)(delta * ENCODER_DIR);

    return encoder_total;
}

/**
 * @brief  Compute one iteration of the PID controller.
 *
 * Standard discrete PID:
 *   error      = target − current
 *   integral  += error × dt
 *   derivative = (error − prev_error) / dt
 *   output     = Kp·error + Ki·integral + Kd·derivative
 *
 * Anti‑windup: integral is clamped to ±INTEGRAL_MAX so that
 * accumulated error during large setpoint changes doesn't cause
 * long overshoot.
 *
 * Dead‑band: if |error| ≤ DEADBAND, output is forced to 0 and
 * integral is cleared.  This prevents the motor from jittering
 * around the target position.
 */
void PID_Compute(void)
{
    /* ----- Read current position ----- */
    pid.current_position = Encoder_GetPosition();

    /* ----- Error ----- */
    pid.error = (float)(pid.target_position - pid.current_position);

    /* ----- Data Recording ----- */
    /* Auto-trigger: start recording when disturbance is detected */
    if (!rec_active && !rec_done &&
        fabsf(pid.error) > (float)RECORD_TRIGGER_THRESHOLD)
    {
        rec_active = 1;
        rec_index  = 0;
    }
    /* Record one sample per PID tick (1 kHz) */
    if (rec_active && !rec_done)
    {
        rec_target[rec_index] = pid.target_position;
        rec_actual[rec_index] = pid.current_position;
        rec_index++;
        if (rec_index >= RECORD_SIZE)
        {
            rec_active = 0;
            rec_done   = 1;   /* ← Check this in debugger. 1 = data ready */
        }
    }

    /* ----- Dead‑band ----- */
    if (fabsf(pid.error) <= (float)DEADBAND)
    {
        pid.output    = 0.0f;
        pid.integral  = 0.0f;
        pid.prev_error = pid.error;
        Motor_SetOutput(0.0f);
        return;
    }

    /* ----- Integral (with anti‑windup clamp) ----- */
    pid.integral += pid.error * PID_DT;
    if (pid.integral >  INTEGRAL_MAX) pid.integral =  INTEGRAL_MAX;
    if (pid.integral < -INTEGRAL_MAX) pid.integral = -INTEGRAL_MAX;

    /* ----- Derivative ----- */
    pid.derivative = (pid.error - pid.prev_error) / PID_DT;

    /* ----- PID output ----- */
    pid.output = pid.kp * pid.error
               + pid.ki * pid.integral
               + pid.kd * pid.derivative;

    /* ----- Save for next iteration ----- */
    pid.prev_error = pid.error;

    /* ----- Apply to motor ----- */
    Motor_SetOutput(pid.output);
}

/**
 * @brief  Apply the PID output to the DRV8838 motor driver.
 *
 * @param  output  Signed PID output.
 *                  > 0 → forward  (PHASE = LOW,  EN = PWM)
 *                  < 0 → reverse  (PHASE = HIGH, EN = PWM)
 *
 * DRV8838 truth table:
 *   nSLEEP=1, PH=0, EN=PWM → OUT1=PWM, OUT2=LOW  (forward)
 *   nSLEEP=1, PH=1, EN=PWM → OUT1=LOW,  OUT2=PWM (reverse)
 */
void Motor_SetOutput(float output)
{
    /* Determine direction */
    if (output >= 0.0f)
    {
        HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_RESET);  /* Forward */
    }
    else
    {
        HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_SET);    /* Reverse */
        output = -output;   /* Make positive for duty cycle */
    }

    /* Clamp to PWM range */
    uint32_t duty = (uint32_t)output;
    if (duty > PWM_MAX) duty = PWM_MAX;

    /* Set PWM duty cycle */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
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
