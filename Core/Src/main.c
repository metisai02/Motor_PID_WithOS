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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stringCut.h"
#include "stdbool.h"
#include "crc16.h"
#include "uart_proto.h"
#include "HanderPID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// #define Pulseee 1980.0 // 11 vong/phut * 45 (ti so truyen 1:45) *  4 (2 canh xung A va B)

// #define PULSE_PER_REVOLUTION  19800
// #define TIME_INTERVAL         0.01f    // Sampling time in seconds
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
/* USER CODE BEGIN PV */
// LiquidCrystal_I2C lcd1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

/* USER CODE BEGIN PFP */
float position, output_pid, output_pid_velo, output_pid_posi;
float error_posi = 0, pre_error_posi = 0;
float error_velo = 0, pre_error_velo = 0;
float now_position, now_encoder_speed, setpoint_posi_degrees;
float number_rotation, velocity_real;
float Kp_true, Ki_true, Kd_true; // truyen tu GUI xuong
uint8_t flagAccept = 0;			 // neu bang 1 thi cho phep dong co chay
uint8_t checkModeFromQt = 0;
uint8_t count_PID = 0;
uint8_t countUpdate = 0;
bool count_PID_position_first_time = true;
uint8_t data_after_cut[PROTO_DATA_SIZE_RX];
PID_control PID_contr;
Select_PID choose_PID;
instance_encoder instance_enc;
// frame
uint16_t get_data_lenght;
uart_proto_handle_t uart_here;
uint16_t frame_tx_lenght;
uint8_t count_test = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

LPF_parameters LPF_para;

// static float now_position1 = 15.23;
// static float velocity_real1 = 21.23;
void send_data_to_Qt();

/**
 *
 */

void send_data_to_Qt()
{
	if (checkModeFromQt == 1 && flagAccept == 1)
	{
		//    sprintf(sendDataToSTM, "%f ", now_position);
		//    HAL_UART_Transmit(&huart1, (uint8_t *)sendDataToSTM, strlen(sendDataToSTM), 200);
		char float_to_char[sizeof(float)];
		char mode = 0x66;
		//	  count_test++;
		//	  if(count_test == 200)
		//	  {
		//		  now_position1 += 5;
		//		  count_test = 0;
		//	  }
		//	  memcpy(float_to_char, &now_position1, sizeof(float));  // real thi bo so 1 ra
		memcpy(float_to_char, &now_position, sizeof(float));
		uint8_t *array_data;
		array_data = (uint8_t *)malloc(5);
		uint8_t index = 0;
		memcpy(array_data + index, &mode, 1); // Mode la 0x66 , neu Qt nhan 0x66 la vi tri
		index += 1;
		memcpy(array_data + index, float_to_char, 4);
		//	  char mang[] = {0x53, 0x53, 0x22, 0x02, 0x65};
		UART_frame_data(array_data, PROTO_DATA_SIZE_TX, uart_here.au8TxBuffer, &frame_tx_lenght);
		//	  UART_frame_data((uint8_t *)mang, PROTO_DATA_SIZE_TX, uart_here.au8TxBuffer, &frame_tx_lenght);
		//	  UART_get_data(&uart_here, uart_here.au8TxBuffer, &get_data_lenght);
		HAL_UART_Transmit_DMA(&huart1, uart_here.au8TxBuffer, frame_tx_lenght);
		free(array_data);
	}
	else if (checkModeFromQt == 2 && flagAccept == 1)
	{
		//    sprintf(sendDataToSTM, "%f ", velocity_real);
		//    HAL_UART_Transmit(&huart1, (uint8_t *)sendDataToSTM, strlen(sendDataToSTM), 200);
		char float_to_char[sizeof(float)];
		char mode = 0x77;
		//	  velocity_real = 20.23;
		//	  count_test++;
		//	  	  if(count_test == 200)
		//	  	  {
		//	  		  velocity_real1 += 5;
		//	  		  count_test = 0;
		//	  	  }
		//	  memcpy(float_to_char, &velocity_real1, sizeof(float));
		memcpy(float_to_char, &velocity_real, sizeof(float));
		uint8_t *array_data;
		array_data = (uint8_t *)malloc(5);
		uint8_t index = 0;
		memcpy(array_data + index, &mode, 1); // Mode la 0x77 , neu Qt nhan 0x77 la velocity
		index += 1;
		memcpy(array_data + index, float_to_char, 4);
		UART_frame_data(array_data, PROTO_DATA_SIZE_TX, uart_here.au8TxBuffer, &frame_tx_lenght);
		HAL_UART_Transmit_DMA(&huart1, uart_here.au8TxBuffer, frame_tx_lenght);
		free(array_data);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		if (data_rx == PROTO_END_BYTE)
		{
			//     data_uart[uart_count++] = '\0';
			data_uart[uart_count++] = data_rx;

			uart_flag = 1; // co ky tu  la chuoi da duoc truyen xong
		}
		else
		{
			data_uart[uart_count] = data_rx;
			uart_count++;
		}
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&data_rx, 1);
	}
}
/*
 tao bien so vong quay, lay so vong quay la position can dien vao GUI, sovongquay =
 */
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&data_rx, 1);
  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// select_mode(Select_Posi);
	while (1)
	{
		//	  a = __HAL_TIM_GET_COUNTER(&htim2);
		//	  HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
		//	  HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET); chieu thuan cung chieu kim dong ho day!!!

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 639;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 639;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;)
	{
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
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	int8_t check_true_false;
	int8_t real_data;
	/* Infinite loop */
	for (;;)
	{
		real_data = 0;
		if (uart_flag == 1)
		{
			memset(data_after_cut, 0, uart_count);
			//    memset(uart_here.au8RxBuffer, 0, PROTO_DATA_SIZE_RX+4);
			memcpy(uart_here.au8RxBuffer, (uint8_t *)data_uart, uart_count);
			check_true_false = UART_get_data(uart_here.au8RxBuffer, uart_count, data_after_cut, &get_data_lenght);
			switch (check_true_false)
			{
			case Phuc_no_valid:
				// todo:
				break;
			case Phuc_false_CRC:
				// todo:
				break;
			case Phuc_buffer_small:
				real_data = 1;
				break;
			case Phuc_false_lenght_data:
				// todo:
				break;
			default:
				// todo:
				break;
			}

			if (real_data == 1)
			{
				while (1) // neu toi duoc day thi đã nhận đúng data rồi
				{
					//   strcpy(data_after_cut, data_uart);
					switch (data_after_cut[0])
					{
					case 0x22:
						//  checkModeFromQt = string_cut_checkMode(data_after_cut);
						setpointQt = *((float *)(data_after_cut + 1));
						checkModeFromQt = 1; // position
						// htim3.Init.Period = 999;
						break;
					case 0x33:
						setpointQt = *((float *)(data_after_cut + 1));
						checkModeFromQt = 2; // velocity
						// htim3.Init.Period = 1999;
						break;
					case 0x55:
						flagAccept = 1;
						break;
					case 0x11:
						Kp_true = *((float *)(data_after_cut + 1));
						Ki_true = *((float *)(data_after_cut + 5));
						Kd_true = *((float *)(data_after_cut + 9));
						break;
					case 0x88:
						if (*((float *)(data_after_cut + 1)) == 1) // PID1
							choose_PID = Select_PID1;
						else if (*((float *)(data_after_cut + 1)) == 2) // PID2
							choose_PID = Select_PID2;
						break;
					case 0x44:
						Kp_true = Ki_true = Kd_true = 0; // nhan nut Reset
						htim2.Instance->CNT = 0;
						instance_enc.position = 0;
						instance_enc.speed_by_encoder = 0;
						setpointQt = 0;
						if (checkModeFromQt == 1)
							control_PID_Position(&PID_contr, setpointQt, Kp_true, Ki_true, Kd_true);
						else if (checkModeFromQt == 2)
							control_PID_Velocity(&PID_contr, setpointQt, Kp_true, Ki_true, Kd_true);

						output_pid = 0;
						HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET); // ko co cai nay dong co no chi dung lai thui chu ko co het keu :)))
						HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
						checkModeFromQt = 0; // do co ham nay = 0, nen phai set output_pid ve 0 luon do no ko nhay vo ham tinh output_pid tu Kp Ki Kd
						flagAccept = 0;
						break;
					default:
						// todo:
						break;
					}
				}
			}
			memset(data_uart, 0, uart_count);
			uart_flag = 0;
			uart_count = 0;
		}

		osDelay(1);
	}
  /* USER CODE END StartTask02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM3)
  	{
  		//		encoder(&htim2,&instance_enc);
  		encoder();
  	}
  	if (checkModeFromQt == 1 && flagAccept == 1 && choose_PID == Select_PID1)
  	{
  		tune_PID_after1(Select_Posi);
  	}

  	else if (checkModeFromQt == 2 && flagAccept == 1 && choose_PID == Select_PID1)
  	{
  		tune_PID_after1(Select_Velo);
  	}

  	else if (checkModeFromQt == 1 && flagAccept == 1 && choose_PID == Select_PID2)
  	{
  		count_PID++;
  		//    countUpdate++;
  		tune_PID_after2(Select_Posi);
  	}
  	else if (checkModeFromQt == 2 && flagAccept == 1 && choose_PID == Select_PID2)
  	{
  		// control_PID_Velocity(&PID_contr, setpointQt, Kp_true, Ki_true, Kd_true);
  		//	  control_PID_Velocity(&PID_contr, 30, 0.7, 1.9, 0.04);
  		//	countUpdate++;
  		tune_PID_after2(Select_Velo);
  	}
  	send_data_to_Qt();
  	//  	control_PID_Velocity(&PID_contr, 40, Kp_true, Kd_true, Ki_true); // toc do 30vong/phut
  	//  	select_mode(Select_Velo);
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
