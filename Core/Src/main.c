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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//typedef struct {
//    uint32_t arr_slow;     // 시작(저속) ARR
//    uint32_t arr_fast;     // 최대(고속) ARR
//    uint32_t ramp_steps;   // 가속/감속에 쓸 스텝 수
//    uint32_t total_steps;  // 이번 이동 총 스텝
//    uint32_t arr_cur;      // 현재 ARR
//} Ramp_t;
typedef struct {
    uint32_t arr_slow;
    uint32_t arr_fast;
    uint32_t ramp_steps;

    uint32_t total_steps;
    uint32_t arr_cur;

    uint32_t diff;         // slow-fast
    uint32_t slope_q16;    // (diff<<16)/R  : 이동 시작 시 1회 계산
} Ramp_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEPS_PER_REV   (1600U)

#define BELT_PITCH_MM        (3.0f)    // GT2면 2.0mm
#define PULLEY_TEETH         (20.0f)   // 예: 20T, 30T 등
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile uint32_t step_count_tim2 = 0;
volatile uint32_t step_count_tim3 = 0;
volatile uint32_t step_count_tim4 = 0;
volatile uint32_t step_count_tim5 = 0;

volatile uint32_t step_target_tim2 = 0;
volatile uint32_t step_target_tim3 = 0;
volatile uint32_t step_target_tim4 = 0;
volatile uint32_t step_target_tim5 = 0;

volatile uint8_t  tim2_running = 0;
volatile uint8_t  tim3_running = 0;
volatile uint8_t  tim4_running = 0;
volatile uint8_t  tim5_running = 0;

// 현재 좌표 저장 및 이동 플래그
volatile int32_t cur_x_mm = 0;
volatile int32_t cur_y_mm = 0;
volatile int32_t cur_z_mm = 0;

volatile int32_t pending_dx_mm = 0;
volatile int32_t pending_dy_mm = 0;
volatile int32_t pending_dz_mm = 0;

volatile uint8_t move_in_progress = 0;

/*가속을 위한 변수*/
volatile Ramp_t ramp_x = { .arr_slow=1999, .arr_fast=199, .ramp_steps=400 }; // 249, 400
volatile Ramp_t ramp_y = { .arr_slow=1999, .arr_fast=199, .ramp_steps=400 };
volatile Ramp_t ramp_z = { .arr_slow=1999, .arr_fast=199, .ramp_steps=400 };

// UART 좌표 통신 =======================================
static uint8_t  rx_ch;

static char     rx_line[64];
static uint32_t rx_idx = 0;

/* 파싱된 좌표 전달용 */
volatile int32_t uart_x, uart_y, uart_z;
volatile uint8_t uart_cmd_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);

/* USER CODE BEGIN PFP */
static inline uint32_t mm_to_steps(float mm);

static void start_x_move_mm(int32_t dx_mm);
static void start_y_move_mm(int32_t dy_mm);
static void start_z_move_mm(int32_t dz_mm);

static void update_motion_done(void);

void MoveToXYZ_mm(int32_t x_mm, int32_t y_mm, int32_t z_mm);

// 가속 함수
static inline void tim_set_arr_ccr50(TIM_HandleTypeDef *htim, uint32_t arr);
static inline void ramp_update(volatile Ramp_t *r, uint32_t step_now, TIM_HandleTypeDef *htim);

// 최적화
static inline void ramp_prepare(volatile Ramp_t *r, uint32_t total_steps);
static inline void ramp_update_fast(volatile Ramp_t *r, uint32_t step_now, TIM_HandleTypeDef *htim);
//static inline void motion_finish_if_all_done_from_isr(void);

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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, &rx_ch, 1);
  // Y1
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);  // ena
  // Y2
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);  // ena
  // X
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);  // ena
  // Z
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);  // ena

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 15;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD3 PD4
                           PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// mm 최적화
//static inline uint32_t mm_to_steps_u32(uint32_t mm_u32)
//{
//    const float mm_per_rev   = (BELT_PITCH_MM * PULLEY_TEETH);
//    const float steps_per_mm = ((float)STEPS_PER_REV) / mm_per_rev;
//
//    float steps_f = (float)mm_u32 * steps_per_mm;
//    uint32_t steps = (uint32_t)(steps_f + 0.5f);
//    return (steps == 0U) ? 1U : steps;
//}

/* 이동 완료 체크 + 좌표 갱신은 Task에서 폴링해서 처리 */
static void update_motion_done(void)
{
	if (!move_in_progress) return;

	// X: TIM4, Y: TIM3+TIM5 모두 끝났는지
	if ((tim2_running == 0) && (tim3_running == 0) && (tim4_running == 0) && (tim5_running == 0))
	{
		cur_x_mm += pending_dx_mm;
		cur_y_mm += pending_dy_mm;
		cur_z_mm += pending_dz_mm;

		pending_dx_mm = 0;
		pending_dy_mm = 0;
		pending_dz_mm = 0;

		move_in_progress = 0;
	}
}

/* x,y 좌표 자유롭게 이동*/
static inline uint32_t mm_to_steps(float mm)
{
    const float mm_per_rev   = (BELT_PITCH_MM * PULLEY_TEETH);
    const float steps_per_mm = ((float)STEPS_PER_REV) / mm_per_rev;
    float steps_f = mm * steps_per_mm;

    uint32_t steps = (uint32_t)(steps_f + 0.5f);
    if (steps == 0) steps = 1;
    return steps;
}

// ---- x축 구동: TIM4
static void start_x_move_mm(int32_t dx_mm)
{
	HAL_TIM_Base_Stop_IT(&htim4);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    if (dx_mm == 0) {
        tim4_running = 0;

        return;
    }

    uint32_t adx = (dx_mm >= 0) ? (uint32_t)dx_mm : (uint32_t)(-dx_mm);
    uint32_t steps = mm_to_steps((float)adx);
    step_target_tim4 = steps;

    step_count_tim4 = 0;
    tim4_running = 1;

    if (dx_mm > 0) {
        // ===== X+ (오른쪽) =====
        // 너가 말한 규칙: X+는 PD1 = SET
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);  // dir
    } else {
        // ===== X- (왼쪽) =====
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);  // dir
    }

//    ramp_x.total_steps = steps;
//    ramp_x.arr_cur     = ramp_x.arr_slow;
    ramp_prepare(&ramp_x, steps);
    tim_set_arr_ccr50(&htim4, ramp_x.arr_cur);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim4);
}

// ---- Y축 구동: TIM3 + TIM5 ----
static void start_y_move_mm(int32_t dy_mm)
{
    if (dy_mm == 0) {
        tim3_running = 0;
        tim5_running = 0;

        HAL_TIM_Base_Stop_IT(&htim3);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(&htim5);
		HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
        return;
    }

    uint32_t ady = (dy_mm >= 0) ? (uint32_t)dy_mm : (uint32_t)(-dy_mm);
    uint32_t steps = mm_to_steps((float)ady);
    step_target_tim3 = steps;
    step_target_tim5 = steps;

    step_count_tim3 = 0;
    step_count_tim5 = 0;

    tim3_running = 1;
    tim5_running = 1;

    if (dy_mm > 0) {
        // ===== Y+ (위쪽) =====
        // 너가 말한 규칙: Y+는 PD5=SET, PD6=SET
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);  // dir
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);  // dir

    } else {
        // ===== Y- (아래쪽) =====
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);  // dir
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);  // dir
    }

//    ramp_y.total_steps = steps;
//    ramp_y.arr_cur     = ramp_y.arr_slow;

    ramp_prepare(&ramp_y, steps);

    // 둘 다 동일 ARR 적용
    tim_set_arr_ccr50(&htim3, ramp_y.arr_cur);
    tim_set_arr_ccr50(&htim5, ramp_y.arr_cur);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim3);

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim5);
}

// ---- Z축 구동: TIM2 ----
static void start_z_move_mm(int32_t dz_mm)
{
    // 안전하게 재시작
    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    if (dz_mm == 0) {
        tim2_running = 0;
        return;
    }

    uint32_t adz = (dz_mm >= 0) ? (uint32_t)dz_mm : (uint32_t)(-dz_mm);
    uint32_t steps = mm_to_steps((float)adz);
    step_target_tim2 = steps;

    step_count_tim2 = 0;
    tim2_running = 1;

    // 요구사항: +방향 SET, -방향 RESET
    if (dz_mm > 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);    // Z+
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);  // Z-
    }

//    ramp_z.total_steps = steps;
//    ramp_z.arr_cur     = ramp_z.arr_slow;

    ramp_prepare(&ramp_z, steps);
    tim_set_arr_ccr50(&htim2, ramp_z.arr_cur);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Base_Start_IT(&htim2);
}

void MoveToXYZ_mm(int32_t x_mm, int32_t y_mm, int32_t z_mm)
{
    if (move_in_progress) return;

    int32_t dx_mm = x_mm - cur_x_mm;
    int32_t dy_mm = y_mm - cur_y_mm;
    int32_t dz_mm = z_mm - cur_z_mm;

    pending_dx_mm = dx_mm;
    pending_dy_mm = dy_mm;
    pending_dz_mm = dz_mm;

    move_in_progress = 1;

    start_x_move_mm(dx_mm);
    start_y_move_mm(dy_mm);
    start_z_move_mm(dz_mm);

    update_motion_done();	// task1 방식일 때 주석 해제, task2일때 주석 처리
}

// 가속 함수
static inline void tim_set_arr_ccr50(TIM_HandleTypeDef *htim, uint32_t arr)
{
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, arr/2); // 50% 듀티
//    __HAL_TIM_SET_COUNTER(htim, 0);  // 위상 튐 현상 발생할 수 있기 떄문에 주석 처리
}

static inline void ramp_update(volatile Ramp_t *r, uint32_t step_now, TIM_HandleTypeDef *htim)
{
    const uint32_t N = r->total_steps;
    if (N == 0) return;

    uint32_t R = r->ramp_steps;
    if (R > 0 && N <= (2U * R)) R = N / 2U;  // 삼각 프로파일

    if (R == 0U) {
        if (r->arr_cur != r->arr_fast) {
            r->arr_cur = r->arr_fast;
            tim_set_arr_ccr50(htim, r->arr_cur);
        }
        return;
    }

    uint32_t s = step_now;
    uint32_t arr;

    if (s <= R) {
    	// 가속: slow -> fast (ARR 감소)
    	uint64_t num = (uint64_t)(r->arr_slow - r->arr_fast) * (uint64_t)s;
    	arr = r->arr_slow - (uint32_t)(num / R);
    } else if (s >= (N - R)) {
    	// 감속: fast -> slow
    	uint32_t k = N - s;
    	uint64_t num = (uint64_t)(r->arr_slow - r->arr_fast) * (uint64_t)k;
    	arr = r->arr_slow - (uint32_t)(num / R);
    } else { // 정속
    	arr = r->arr_fast;
    }

    if (arr != r->arr_cur) {
        r->arr_cur = arr;
        tim_set_arr_ccr50(htim, arr);
    }
}

// ====== 램프 준비: 이동 시작 시 1회만 계산 ======
static inline void ramp_prepare(volatile Ramp_t *r, uint32_t total_steps)
{
    r->total_steps = total_steps;
    r->arr_cur     = r->arr_slow;

    // diff = slow - fast (ARR 감소가 가속)
    r->diff = (r->arr_slow > r->arr_fast) ? (r->arr_slow - r->arr_fast) : 0U;

    // 삼각 프로파일 처리: N <= 2R 이면 R_eff = N/2
    uint32_t R_eff = r->ramp_steps;
    if (R_eff > 0U && total_steps <= (2U * R_eff)) {
        R_eff = total_steps / 2U;
    }

    // R_eff가 0이면 slope=0
    if (R_eff == 0U || r->diff == 0U) {
        r->slope_q16 = 0U;
        return;
    }

    // (diff<<16)/R_eff : 여기서만 나눗셈 1회
    r->slope_q16 = (uint32_t)(((uint64_t)r->diff << 16) / (uint64_t)R_eff);
}

// ====== 램프 업데이트(고속): ISR에서 호출, 나눗셈 없음 ======
static inline void ramp_update_fast(volatile Ramp_t *r, uint32_t step_now, TIM_HandleTypeDef *htim)
{
    const uint32_t N = r->total_steps;
    if (N == 0U) return;

    // slope가 0이면(=R_eff 0이거나 diff 0), 그냥 fast로 고정
    if (r->slope_q16 == 0U) {
        if (r->arr_cur != r->arr_fast) {
            r->arr_cur = r->arr_fast;
            tim_set_arr_ccr50(htim, r->arr_cur);
        }
        return;
    }

    // 다시 R_eff 계산(여긴 나눗셈 없음)
    uint32_t R_eff = r->ramp_steps;
    if (R_eff > 0U && N <= (2U * R_eff)) {
        R_eff = N / 2U;
    }
    if (R_eff == 0U) {
        // 극단 케이스
        if (r->arr_cur != r->arr_fast) {
            r->arr_cur = r->arr_fast;
            tim_set_arr_ccr50(htim, r->arr_cur);
        }
        return;
    }

    uint32_t s = step_now;
    uint32_t arr;

    if (s <= R_eff) {
        // 가속: slow -> fast
        uint32_t dec = (uint32_t)(((uint64_t)s * (uint64_t)r->slope_q16) >> 16);
        arr = r->arr_slow - dec;
    } else if (s >= (N - R_eff)) {
        // 감속: fast -> slow
        uint32_t k = N - s;
        uint32_t dec = (uint32_t)(((uint64_t)k * (uint64_t)r->slope_q16) >> 16);
        arr = r->arr_slow - dec;
    } else {
        // 정속
        arr = r->arr_fast;
    }

    if (arr != r->arr_cur) {
        r->arr_cur = arr;
        tim_set_arr_ccr50(htim, arr);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == huart3.Instance) // 또는 huart->Instance == USART1 이런 식
    {
        char c = (char)rx_ch;

        if (c == '\r') {
            // ignore
        }
        else if (c == '\n')
        {
            rx_line[rx_idx] = '\0';
            rx_idx = 0;

            // 여기서 파싱은 "가볍게"만 하고, 실제 MoveTo는 Task에서 처리 권장
            int32_t x,y,z;
            // 포맷: G x y z
            if (sscanf(rx_line, "G %ld %ld %ld", &x, &y, &z) == 3)
            {
                uart_x = x; uart_y = y; uart_z = z;
                uart_cmd_ready = 1;
            }
        }
        else
        {
            if (rx_idx < (sizeof(rx_line) - 1)) {
                rx_line[rx_idx++] = c;
            } else {
                // overflow 방지: 리셋
                rx_idx = 0;
            }
        }

        // 다음 바이트 재수신
        HAL_UART_Receive_IT(&huart3, &rx_ch, 1);
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	  MoveToXYZ_mm(10, 10, 10);
//	  while (move_in_progress) { update_motion_done(); osDelay(1); }
//	  osDelay(1000);
//
//	  MoveToXYZ_mm(200, 300, 200);
//	  while (move_in_progress) { update_motion_done(); osDelay(1); }
//	  osDelay(1000);
//
//	  MoveToXYZ_mm(150, 200, 100);
//	  while (move_in_progress) { update_motion_done(); osDelay(1); }
//	  osDelay(2000);
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  if (uart_cmd_ready)
	  {
		  uart_cmd_ready = 0;

		  MoveToXYZ_mm(uart_x, uart_y, uart_z);

		  // 이동 완료까지 대기(너 코드 방식 유지)
		  while (move_in_progress) {
			  update_motion_done();
			  osDelay(1);
		  }
	  }
	  osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  /* x, y, z 좌표*/
	// ----- Z (TIM2) -----
	if (htim->Instance == TIM2 && tim2_running)
	{
		step_count_tim2++;
//		ramp_update(&ramp_z, step_count_tim2, &htim2);
		ramp_update_fast(&ramp_z, step_count_tim2, &htim2);

		if (step_count_tim2 >= step_target_tim2)
		{
			tim2_running = 0U;
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		}
	    return;
	}
	// ----- Y (TIM3) -----
	if (htim->Instance == TIM3 && tim3_running)
	{
		step_count_tim3++;
//		ramp_update(&ramp_y, step_count_tim3, &htim3);
		ramp_update_fast(&ramp_y, step_count_tim3, &htim3);
//		tim_set_arr_ccr50(&htim5, ramp_y.arr_cur);   // ✅ Y2도 같은 ARR로 강제 동기

		if (step_count_tim3 >= step_target_tim3)
		{
			tim3_running = 0U;
			HAL_TIM_Base_Stop_IT(&htim3);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		}
		return;
	}

	// ----- X (TIM4) -----
	if (htim->Instance == TIM4 && tim4_running)
	{
		step_count_tim4++;
//		ramp_update(&ramp_x, step_count_tim4, &htim4);
		ramp_update_fast(&ramp_x, step_count_tim4, &htim4);

		if (step_count_tim4 >= step_target_tim4)
		{
			tim4_running = 0U;
			HAL_TIM_Base_Stop_IT(&htim4);
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
		}
	    return;
	}

	// ----- Y (TIM5) -----
	if (htim->Instance == TIM5 && tim5_running)
	{
		step_count_tim5++;
//		ramp_update(&ramp_y, step_count_tim5, &htim5);
		ramp_update_fast(&ramp_y, step_count_tim5, &htim5);

	    // 램프 계산은 TIM3에서만 했으니 여기선 종료만
	    if (step_count_tim5 >= step_target_tim5)
		{
			tim5_running = 0;
			HAL_TIM_Base_Stop_IT(&htim5);
			HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
		}
	    return;
	}
  /* USER CODE END Callback 1 */
}

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
