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
#include "stringCut.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Pulseee 1980.0 // 11 vong/phut * 45 (ti so truyen 1:45) *  4 (2 canh xung A va B)
#define Ts 0.05
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

/* USER CODE BEGIN PV */
// LiquidCrystal_I2C lcd1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
uint32_t timeGetTick;
int16_t vongtrengiay = 0, checkxung = 0, speed = 0;
float position, output_pid;
float error_posi = 0, pre_error_posi = 0;
float error_velo = 0, pre_error_velo = 0;
volatile short now_counter = 0;
volatile int32_t motor_position;
float now_position, now_position1, now_encoder_speed, setpoint_posi_degrees;
float number_rotation, velocity_real;
char data1[30] = "Position is:";
char buffer[50];
float Kp_true, Ki_true, Kd_true;   //truyen tu GUI xuong
typedef struct
{
  float P_part;
  float I_part;
  float D_part;
} PID_control;
PID_control PID_contr;

typedef struct
{
  int32_t position;
  //	int32_t position_real;
  int16_t speed_by_encoder; // don vi: xung/(tgian ngat timer)
  int16_t pre_speed_by_encoder;
  short pre_counter;
  int32_t velocity; // vong/phut
  //	volatile float velocity_degrees_p_sec;
  //	volatile float velocity_not;   // bien nay dung trong truong hop quay nguoc dan den toc do am, dung chung lun
} instance_encoder;
instance_encoder instance_enc;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum
{
  RR_MOTOR,
  NRR_MOTOR,
  STOP_MOTOR
} motor_status;
motor_status motor;

typedef enum
{
  Select_Velo,
  Select_Posi,
} Select_Tune;
int32_t prev_count = 0;
void send_data_to_Qt();
// Select_Tune select_tunning;
// void motor_control(float check_error ,uint16_t duty)
//{
////	xemlun = cc;
////	caidutyday = duty/100;
////	real = duty*(htim1.Instance->ARR)/100;
////	switch(check_error)
////	{
////		case NRR_MOTOR:
//	if(check_error)
////			HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
////			HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,1);
//			HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,0);
//			htim1.Instance->CCR3 = duty*(htim1.Instance->ARR)/100;;
////			htim1.Instance->CCR3 = duty*999/100;
//		break;
//		case RR_MOTOR:
////			HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
//			HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_RESET);
////			HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,0);
//			HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,1);
////			htim1.Instance->CCR3 = htim1.Instance->ARR - duty*(htim1.Instance->ARR)/100;;
//			htim1.Instance->CCR3 = duty*(htim1.Instance->ARR)/100;
//		break;
//		case STOP_MOTOR:
//			HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_RESET);
//			htim1.Instance->CCR3 = 0;
//		}
//}
void PWM_control_position(TIM_HandleTypeDef *htim, float duty)
{
  //	if(duty>90.0)
  //	{
  //		duty = 90.0;
  //	}
  if (duty > 0)
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET); // chieu thuan cung chieu kim dong ho
    htim1.Instance->CCR3 = duty * (htim1.Instance->ARR) / 100;

    ;
    //		htim1.Instance->CCR3 =  duty*900/100;
  }
  else if (duty < 0)
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    htim1.Instance->CCR3 = (-duty) * (htim1.Instance->ARR) / 100; // nguoc chieu kim dong ho
                                                                  //		htim1.Instance->CCR3 =  -duty*900/100;
  }
  else
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

  }
}
void PWM_control_velocity(TIM_HandleTypeDef *htim, float duty)
{
  if (duty > 0)
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET); // chieu thuan cung chieu kim dong ho
    htim1.Instance->CCR3 = duty * (htim1.Instance->ARR) / 100;
    ;
  }
  else if (duty < 0)
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    htim1.Instance->CCR3 = (-duty) * (htim1.Instance->ARR) / 100; // nguoc chieu kim dong ho
  }
  else
  {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
  }
}
// void encoder(TIM_HandleTypeDef *htim, instance_encoder *encoder_value)
//{     // tinh toc do xung moi 100ms
//
//	 now_counter = __HAL_TIM_GetCounter(htim);    // now_counter = CNT hien tai
//		if(now_counter == (encoder_value -> pre_counter))
//		{
//			encoder_value -> velocity = 0;
//		}
//		else if(now_counter > (encoder_value -> pre_counter)) // trong truong hop gia tri counter sau > counter truoc
//		{
//			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))   // neu timer dang dem xuong, dem xuogn tuc la quay nguoc, quay nguoc thi toc do am
//			{
//				encoder_value -> speed_by_encoder = -encoder_value -> pre_counter - (__HAL_TIM_GET_AUTORELOAD(htim) - now_counter); //hal_tim_reload la dung de lay gia tri ARR ra
//				encoder_value -> velocity = (float)(encoder_value -> speed_by_encoder) * 60.0f / Tsf / Pulseee;   // Ts la thoi gian ngat cai trong cube, phai sua lai thanh tgian lay mau chu de vay chinh mac cong lam
//			}  //tran`, dang dem xuong -> counter hien tai la: -150, counter truoc la -65333 --> hien tai > truoc --> toc do: --65333 - (65536 - -150)
//			else //dem len
//			{
//				encoder_value -> speed_by_encoder = now_counter - (encoder_value->pre_counter);
//				encoder_value -> velocity = (float)(encoder_value -> speed_by_encoder) * 60.0f / Tsf / Pulseee;
//			}
//		}
//		else   // counter hien tai < counter truoc
//		{
//			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
//			{
//				encoder_value->speed_by_encoder = now_counter - (encoder_value->pre_counter);
//				encoder_value -> velocity = (float)(encoder_value -> speed_by_encoder) * 60.0f / Ts / Pulseee;
//			}  // ko tran` , counter hien tai : 150 , counter truoc do 350 --> toc do: 150-350, toc do am do quay nguoc chieu`
//			else  // dem len
//			{
//				encoder_value->speed_by_encoder = now_counter + (__HAL_TIM_GetAutoreload(htim) - (encoder_value->pre_counter));
//				encoder_value -> velocity = (float)(encoder_value -> speed_by_encoder) * 60.0f / Ts / Pulseee;
//			} // khi tran`, dang dem len --> counter : truoc la 65533 , sau la 105 thi toc do la 105+(65536 - 65533)
//		}
//		encoder_value->velocity_degrees_p_sec = (encoder_value->velocity) * 360.0/60.0;      /* 1s thi duoc .... vong --> 100ms thi duoc so vong la
//																								velocity_degrees_p_sec*(tgian ngat Ts)  , o day velocity_degrees_p_sec
//																								la tinh do/s nen sau do phai nhan Ts (tgian ngat timer) de tinh vi tri*/
//		encoder_value -> position += (encoder_value->velocity_degrees_p_sec)*Ts;
////		if(encoder_value -> position >= 360.0)
////		{
////			count1++;
////			encoder_value -> position = count1*360.0 + ((int32_t)(encoder_value -> position) % 360.0);
////		}
////		else if(encoder_value -> position <= -360.0)
////		{
////			count2++;
////			encoder_value -> position = count2*(-360.0) + ((int32_t)(encoder_value -> position) % 360.0);
////		}
//											/* Tss: 2do/Tss   --> tai Tss : 2do
//										 	 0.2s: 3do/Tss		--> tai 0.2s : 5do
//											 0.3s: 5do/Tss		--> tai 0.3s : 10do    */
//
//		encoder_value -> pre_counter = now_counter;
//}  // velocity:   // vong/phut

void encoder()
{
  instance_enc.speed_by_encoder = htim2.Instance->CNT - instance_enc.pre_speed_by_encoder;
  //	htim2.Instance->CNT = 0;
  instance_enc.pre_speed_by_encoder = htim2.Instance->CNT;
  //	instance_enc.speed_by_encoder = htim2.Instance->CNT;
  instance_enc.position += instance_enc.speed_by_encoder;
  //	htim2.Instance->CNT = 0;

}
// void control_PID_velocity(PID_control *pid_tune, float setpoint, float Kp, float Ki, float Kd)
// {
//   error = setpoint - instance_enc.velocity;
//   pid_tune->P_part = error;
//   pid_tune->I_part += error * Ts;
//   //	if(error < 0.03*setpoint)
//   //	{
//   //		pid_tune->I_part = 0;
//   //	}
//   pid_tune->D_part = (error - pre_error) / Ts;
//   output_pid = Kp * (pid_tune->P_part) + Ki * (pid_tune->I_part) + Kd * (pid_tune->D_part);
//   if (output_pid > 90.0)
//   {
//     output_pid = 90.0;
//   }
//   else if (output_pid < 0)
//   {
//     output_pid = 0;
//   }
//   pre_error = error;
//   //}
/**
 * 
*/
void send_data_to_Qt()
{
  sprintf(sendDataToSTM, "Kp:%f | Ki:%f | Kd:%f\n", Kp_true, Ki_true, Kd_true);
  HAL_UART_Transmit(&huart1, (uint8_t *)sendDataToSTM, strlen(sendDataToSTM), 200);
}
void control_PID_Position(PID_control *pid_tune, float setpoint_posi_rotation, float Kp, float Ki, float Kd) // moi chi dieu khien duoc toc do dong co
{
  //	instance_enc.velocity_not = instance_enc.velocity;
  //	if(instance_enc.velocity_not < 0)
  //	{
  //		instance_enc.velocity_not = -instance_enc.velocity_not;   // am thi doi thanh duong cho de dung PID =))))
  //	}
  now_position = (float)instance_enc.position * 360 / 1980; // now_position = độ
  number_rotation = now_position / 360;
  //	setpoint_posi_degrees = setpoint_posi_rotation*360;   // setpoint_posi_rotation la set số vòng cho dễ set
  //	now_position1 = 0.85*now_position1 + 0.15*now_position;
  error_posi = setpoint_posi_rotation - (now_position);
  pid_tune->P_part = error_posi;
  pid_tune->I_part += error_posi * Ts;
  //	if(error < 0.03*setpoint)
  //	{
  //		pid_tune->I_part = 0;
  //	}
  pid_tune->D_part = (error_posi - pre_error_posi) / Ts;
  output_pid = Kp * (pid_tune->P_part) + Ki * (pid_tune->I_part) + Kd * (pid_tune->D_part);
  if (output_pid > 90.0)
  {
    output_pid = 90.0;
  }
  else if (output_pid < -90)
  {
    output_pid = -90.0;
  }
  //	else if(output_pid < 0)
  //	{
  //		output_pid = 0;
  //	}
  pre_error_posi = error_posi;
}
void control_PID_Velocity(PID_control *pid_tune, float setpoint_velo, float Kp, float Ki, float Kd) // moi chi dieu khien duoc toc do dong co
{                                                                                                   // velocity vong/phut
  velocity_real = (float)instance_enc.speed_by_encoder * 60.0f / (Ts * Pulseee);
  error_velo = setpoint_velo - (velocity_real);
  instance_enc.velocity = velocity_real;
  pid_tune->P_part = error_velo;
  pid_tune->I_part += error_velo * Ts;
  pid_tune->D_part = (error_velo - pre_error_velo) / Ts;
  output_pid = Kp * (pid_tune->P_part) + Ki * (pid_tune->I_part) + Kd * (pid_tune->D_part);
  if (output_pid > 90.0)
  {
    output_pid = 90.0;
  }
  else if (output_pid < -90)
  {
    output_pid = -90.0;
  }
  pre_error_velo = error_velo;
}
void select_mode(Select_Tune select)
{
  //	select_tunning select;
  switch (select)
  {
  case Select_Posi:
    PWM_control_position(&htim1, output_pid);
    //			sprintf(buffer, "Position is: %.3f", now_position);
    //			HAL_UART_Receive_IT(&huart1, buffer, strlen(buffer));
    break;
  case Select_Velo:
    PWM_control_velocity(&htim1, output_pid);
    break;
  default:
    break;
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    //		encoder(&htim2,&instance_enc);
    encoder();
  }
  control_PID_Position(&PID_contr, 0.5 * 360, Kp_true, Ki_true, Kd_true);
  select_mode(Select_Posi);
  send_data_to_Qt();
//  	control_PID_Velocity(&PID_contr, 40, Kp_true, Kd_true, Ki_true); // toc do 30vong/phut
//  	select_mode(Select_Velo);
}  


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	  {
	    if (data_rx == '\n')
	    {
	      data_uart[uart_count] = '\0';
	      uart_flag = 1; // co ky tu \n la chuoi da duoc truyen xong
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

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // select_mode(Select_Posi);
  while (1)
  {
    //	  a = __HAL_TIM_GET_COUNTER(&htim2);
    //	  HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
    //	  HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET); chieu thuan cung chieu kim dong ho day!!!
	  if (uart_flag == 1)
	      {
	        memset(data_recFromPC, 0, uart_count);
	        strcpy(data_recFromPC, data_uart);
	        //		  string_cut(data_recFromPC);
	        Kp_true = string_cut(data_recFromPC, "Kp");
	        Ki_true = string_cut(data_recFromPC, "Ki");
	        Kd_true = string_cut(data_recFromPC, "Kd");
	        memset(data_uart, 0, uart_count);
	        uart_count = 0;
	        uart_flag = 0;
	      }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  htim1.Init.Prescaler = 63;
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
  htim3.Init.Period = 4999;
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
