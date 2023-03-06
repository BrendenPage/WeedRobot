/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdbool.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SPRAYER_PIN_CLASS   GPIOB
#define SPRAYER_CONTROL_PIN GPIO_PIN_12
#define SPRAYER_DELAY         1000

// These defines list the pins for the
// brushless motors that drive the chassis
#define MOTOR_HALT      0
#define MOTOR_F         1
#define MOTOR_FS        2
#define MOTOR_B         3
#define MOTOR_BS        4


#define CAMERA_HALT	    2000
#define IGNORE_DELAY    2000
#define RESET_COUNT     5000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint32_t VR;
uint8_t dma_tx_buffer[BUFFER_SIZE];
uint8_t dma_rx_buffer[BUFFER_SIZE];
//uint8_t finished_turn;
//uint8_t handled_detected_object;
uint8_t global_status;
int direction;
float degree;
uint16_t spray_counter_1;
uint16_t spray_counter_2;
uint16_t reset_counter;
bool start_turn;
bool spray_status;
int turn_counter;



// Global variable that determines the state of the
// motors
// 0: halt, 1: forward, 2: backward
//volatile uint8_t MOTOR_STATE_VAR;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */



void stepper_half_drive(int step);
void stepper_step_angle (float angle, int direction);
void motor_state_selector(int state);
void loop_motor_wait(int ticks);

// Takes a state input and sets the motor pins
// to execute the movement as follows:
// 0 - halt
// 1 - forward
// 2 - backward
// 3 - left
// 4 - right
void change_motor_state(short state);

// Takes the arm motor's current state and returns the
// updated state with respect to the ARM global variable
int check_aim(int state);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  memcpy(dma_tx_buffer, dma_rx_buffer, BUFFER_SIZE);
  if (global_status == 1)
  {
      //Check for Stepper motor command
	  if(dma_tx_buffer[0] == 'R')
	  {
		  direction = 1;
		  degree = 10.0;
		  spray_counter_1 = 0;
		  start_turn = true;
	  }
	  else if(dma_tx_buffer[0] == 'r')
	  {
		  direction = 1;
		  degree = 5.0;
		  spray_counter_1 = 0;
		  start_turn = true;
	  }
	  else if(dma_tx_buffer[0] == 'L')
	  {
		  direction = 0;
		  degree = 10.0;
		  spray_counter_1 = 0;
		  start_turn = true;
	  }
	  else if(dma_tx_buffer[0] == 'l')
	  {
		  direction = 0;
		  degree = 5.0;
		  spray_counter_1 = 0;
		  start_turn = true;
	  }
	  else if (dma_tx_buffer[0] == 'H')
	  {
		  direction = 2;
		  degree = 0.0;
		  spray_counter_1++;
		  start_turn = true;
	  }
	  else
	  {
		  direction = 2;
		  degree = 0.0;
		  reset_counter ++;
	  }



	  if(dma_tx_buffer[1] == 'F')
	  {
		  motor_state_selector(MOTOR_F);
		  spray_counter_2 = 0;

	  }
	  else if(dma_tx_buffer[1] == 'S')
	  {
		  motor_state_selector(MOTOR_FS);
		  spray_counter_2 = 0;
	  }
	  else if(dma_tx_buffer[1] == 'H')
	  {
		  motor_state_selector(MOTOR_HALT);
		  spray_counter_2 ++ ;
	  }
	  else
	  {
		  motor_state_selector(MOTOR_HALT);
	  }

  }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_DMA(&huart2, dma_rx_buffer, BUFFER_SIZE);
}

void DMA_Message_Init(void)
{
	uint8_t i;
	for(i = 0; i < BUFFER_SIZE; i++)
	{
		dma_rx_buffer[i] = 'M';
	}
}

void Counter_Init(void)
{
	spray_counter_1 = 0;
	spray_counter_2 = 0;
	reset_counter = 0;
	turn_counter = 0;
}

void Global_Status_Init(void)
{
	global_status = 0;
	spray_status = false;
}

void Stepper_Motor_Reset(void)
{
	if(turn_counter < 0)
	{
		direction = 0;
	}
	else
	{
		direction = 1;
	}

	degree = (float)(turn_counter * 5);
	stepper_step_angle(degree, direction);
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
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc3, &VR, 1);
  DMA_Message_Init();
  HAL_UART_Receive_DMA(&huart2, dma_rx_buffer, BUFFER_SIZE);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  Global_Status_Init();
  Counter_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (global_status == 1)
	  {
		  stepper_step_angle(degree, direction);
		  if(direction == 0)
		  {
			  if(degree == 10.0)
			  {
				  turn_counter  = turn_counter + 2;
			  }
			  else if(degree == 5.0)
			  {
				  turn_counter  = turn_counter + 1;
			  }
		  }
		  else if (direction == 1)
		  {
			  if(degree == 10.0)
			  {
				  turn_counter  = turn_counter - 2;
			  }
			  else if(degree == 5.0)
			  {
				  turn_counter  = turn_counter - 1;
			  }
		  }

	  }
	  else if (global_status == 2)
	  {
		  int i;
		  HAL_GPIO_WritePin(SPRAYER_PIN_CLASS, SPRAYER_CONTROL_PIN, GPIO_PIN_SET);
		  for (i = 0; i < SPRAYER_DELAY; i ++);
		  HAL_GPIO_WritePin(SPRAYER_PIN_CLASS, SPRAYER_CONTROL_PIN, GPIO_PIN_RESET);
		  Stepper_Motor_Reset();
		  motor_state_selector(MOTOR_F);
		  for (i = 0; i < IGNORE_DELAY; i ++);
		  spray_status = true;
	  }
	  else
	  {
		  motor_state_selector(5);
	  }


	  if (global_status == 0 && (dma_tx_buffer[0] != 'M' || dma_tx_buffer[1] != 'M' ))
	  {
		  global_status = 1;
	  }

	  if (global_status == 1 && reset_counter > RESET_COUNT)
	  {
		  global_status = 0;
	  }

	  if (global_status == 1 && spray_counter_1 > CAMERA_HALT && spray_counter_2 > CAMERA_HALT)
	  {
		  global_status = 2;
	  }

	  if (global_status == 2 && spray_status == 1)
	  {
		  global_status = 0;
		  spray_status = false;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	if(finished_turn == 0)
//	{
//		stepper_step_angle(degree, direction);
//		dma_rx_buffer[0] = 'H';
//		dma_rx_buffer[1] = 'M';
//		finished_turn = 1;
//	}
//	if (spray_counter != -1 && spray_counter != 0) spray_counter--;
//    if (spray_counter == 0) {
//      // Turn on the sprayer
//      spray_counter = -1;
//      HAL_GPIO_WritePin(SPRAYER_PIN_CLASS, SPRAYER_CONTROL_PIN, GPIO_PIN_SET);
//      sprayer_delay = SPRAYER_DELAY;
//    } else if (sprayer_delay == 0) {
//      // Turn off the sprayer
//      HAL_GPIO_WritePin(SPRAYER_PIN_CLASS, SPRAYER_CONTROL_PIN, GPIO_PIN_RESET);
//      MOTOR_STATE_VAR = 1;
//    } else {
//    	sprayer_delay--;
//    }
//
//    if (1) {
//    	motor_state_selector(MOTOR_STATE_VAR);
//    	motor_delay = MOTOR_DELAY;
//    }

    HAL_Delay(1);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4095;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 4095;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim6.Init.Prescaler = 7199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC15 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void loop_motor_wait(int ticks) {
  int step = 0;
  while (ticks != 0) {
    stepper_half_drive(step);
    // Get ADC value
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 1);
    VR = HAL_ADC_GetValue(&hadc3);
    if (VR < 1365) {
      if (step == 0) {
        step = 7;
      } else {
        step--;
      }
    } else if (VR > 1365*2) {
      step = (step + 1) % 8;
    }
    HAL_Delay(1);
    ticks--;
  }
}


void motor_state_selector(int state) {
	switch(state){
	  case 0:
		  // Halt
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);// PIN B15 off
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);// PIN B15 off
		  return;
	  case 1:
		  // Forward
		  // 4095 correlates with max speed (100% duty cycle) in the forward direction,
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 4095);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 4095);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);// PIN B15 off
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);// PIN B14 off
		  return;
	  case 2:
		  // Forward slowly
		  // 4095 correlates with max speed (100% duty cycle) in the forward direction,
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2048);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2048);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);// PIN B15 off
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);// PIN B14 off
		  return;
	  case 3:
		  // Backward
		  // 2048 correlates with max speed (50% duty cycle) in the backward direction,
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 4095);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 4095);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);// PIN B15 on
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);// PIN B14 on
		  return;
	  case 4:
		  // Backward
		  // 2048 correlates with max speed (50% duty cycle) in the backward direction,
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2048);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2048);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);// PIN B15 on
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);// PIN B14 on
		  return;
	}
}


// BLOCKING OPERATION, TAKES <<ANGLE>> MILLISECONDS TO COMPLETE
void stepper_step_angle (float angle, int direction)
{
	float anglepersequence = 0.703125;  // 360 = 512 sequences
	int numberofsequences = (int) (angle/anglepersequence);

	for (int seq=0; seq<numberofsequences; seq++)
	{
		if (direction == 0)  // for clockwise
		{
			for (int step=7; step>=0; step--)
			{
				stepper_half_drive(step);
        HAL_Delay(1);
				// stepper_set_rpm(rpm);
			}

		}

		else if (direction == 1)  // for anti-clockwise
		{
			for (int step=0; step<8; step++)
			{
				stepper_half_drive(step);
        HAL_Delay(1);
				// stepper_set_rpm(rpm);
			}
		}
	}
}

void stepper_half_drive (int step)
{
  switch (step){
    case 0:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   // IN2
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);   // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);   // IN4
        break;

    case 1:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // IN2
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);   // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);   // IN4
        break;

    case 2:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // IN2
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);   // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);   // IN4
        break;

    case 3:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);   // IN2
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);   // IN4
        break;

    case 4:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   // IN2
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);   // IN4
        break;

    case 5:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   // IN2
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);   // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);   // IN4
        break;

    case 6:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);   // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   // IN2
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);   // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);   // IN4
        break;

    case 7:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // IN1
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   // IN2
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);   // IN3
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);   // IN4
        break;

    }
}


//void change_motor_state(short state)
//{
//  switch(state){
//    case 0:
//    // Halt
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_FORWARD_1, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_REVERSE_1, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_FORWARD_2, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_REVERSE_2, GPIO_PIN_RESET);
//    case 1:
//    // Forward
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_FORWARD_1, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_REVERSE_1, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_FORWARD_2, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_REVERSE_2, GPIO_PIN_RESET);
//    case 2:
//    // Backward
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_FORWARD_1, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_REVERSE_1, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_FORWARD_2, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_REVERSE_2, GPIO_PIN_SET);
//    case 3:
//    // Left
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_FORWARD_1, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_REVERSE_1, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_FORWARD_2, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_REVERSE_2, GPIO_PIN_RESET);
//    case 4:
//    // Left
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_FORWARD_1, GPIO_PIN_SET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_REVERSE_1, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_FORWARD_2, GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(MOTOR_PIN_CLASS, MOTOR_REVERSE_2, GPIO_PIN_SET);
//  }
//}

//int check_aim(int state) {
//  // Get ADC value
//  if (AIM == 1) {
//    if (state == 0) {
//      state = 7;
//    } else {
//      state--;
//    }
//  } else if (AIM == 2) {
//    state = (state + 1) % 8;
//  }
//  return state;
//}



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
