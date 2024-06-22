/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PPR 920
#define Time_Interrupt 0.001
#define V_max 150
#define A_max 1
#define gear_ratio 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
float sum_A[20] , sum_B[20];
volatile float CurvelA,CurvelB;
uint32_t EncoderA , EncoderA_pre , EncoderB , EncoderB_pre;
int32_t deltaA, deltaB;
/////PID parameters/////
//Setpoint
volatile float SP_WL ;
volatile float SP_WR ;

float Kp_R = 3.8;
float Ki_R = 4;
float Kd_R = 0;

float Kp_L = 3.7;
float Ki_L = 4;
float Kd_L = 0;

 volatile float ek_R, ek_L , ek_R1,ek_L1;
 volatile float Up_R,Ui_R,Ud_R,Ui_1_R,U_out_R;
 volatile float Up_L,Ui_L,Ud_L,Ui_1_L,U_out_L;

 double TempPWM_L, TempPWM_R;
 uint16_t OutPWM_L, OutPWM_R;
 /////Trapezoidal parameters/////
 float Target_pos;
 volatile float current_position = 0;
 volatile float current_velocity = 0;
 volatile double WL , WR;
 volatile float t,t1, t2, t_total;
 float T = 0.01;
 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
float Average_5_times(float Var, float Temp[20]);
float PID_WL (float SP, float Vel);
float PID_WR (float SP, float Vel);
float Anti_windup (float Temp , float Kb, uint16_t High, uint16_t Low);
void rotateDC_L ( int8_t Direct );
void rotateDC_R ( int8_t Direct );
void get_position(char *s, int *cur_x, int *cur_y);
float Trapezoidal_Velocity (float target_position, float A, float V, float T);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Average_5_times(float Var, float Temp[20])
{
    float sum = 0, Out_Average_Var; // Initialize sum to 0
    for (int i = 0; i < 19; i++)
    {
        Temp[i] = Temp[i + 1];  // g�n gi� tr? hi?n t?i v�o gi� tr? tru?c
        sum += Temp[i];        // C?ng d?n c�c gi� tr? v?a luu
    }
    // G�n gi� tr? m?i nh?t
    Temp[19] = Var;
    sum += Temp[19] ;
    // T�nh trung b�nh
    Out_Average_Var = sum / 20;
    return Out_Average_Var;
}

float PID_WL (float SP , float Vel)
{
   ek_L = SP -  Vel;
	 Up_L = Kp_L * ek_L;
	 Ui_L = Ui_1_L + Ki_L * ek_L * Time_Interrupt;
	 U_out_L = Up_L + Ui_L   ;
   Ui_1_L = Ui_L;

  return U_out_L;
}

float PID_WR (float SP , float Vel)
{
   ek_R = SP - Vel;
	 Up_R = Kp_R * ek_R;
	 Ui_R = Ui_1_R + Ki_R * ek_R * Time_Interrupt;
	 U_out_R = Up_R + Ui_R ;
	 Ui_1_R = Ui_R;

   return U_out_R;
}

float Anti_windup (float Temp , float Kb , uint16_t High , uint16_t Low)
{
  float e_reset = 0;
  float Ui_anti;

  if (Temp > High)
  {
    e_reset = (High = Temp);
  }
  else if (Temp < Low)
  {
    e_reset = (Low - Temp);
  }
  else
  {
    e_reset = 0;
  }

  Ui_anti = Time_Interrupt * e_reset * Kb;

  return Ui_anti;

}

void rotateDC_L ( int8_t Direct )
{
	if (Direct == 1 )
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,GPIO_PIN_SET);
	}
	
	if (Direct == -1 )
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,GPIO_PIN_RESET);
	}
}

void rotateDC_R ( int8_t Direct )
{
	if (Direct == -1 )
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_SET);
	}
	
	if (Direct == 1 )
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET);
	}
}


void get_position(char *s, int *cur_x, int *cur_y) {
    char *commaPos = strchr(s, ',');
    char *colonPos = strchr(s, ':');
    *cur_x = atoi(colonPos + 2);
    colonPos = strchr(commaPos, ':');
    *cur_y = atoi(colonPos + 2);
}

void Trapezoidal_Velocity ()
{
	  A = 4 * (Target_pos - current_position)/(t_total*t_total);
	 t1 = (t_total /2) - 0.5 * sqrt((t_total*t_total) - (4 * abs(Target_pos - current_position)/A));
	for (int i =0; i <= (t_total/T);i++)
	{ 
		t = i * T;
		if (t <= t1 && t >=0 )
		{
			qt = current_position + 0.5 * A * t * t;
			qt_dot = A * t;
		}
		else if ( t > t1 && t <= t_total - t1)
		{
			qt = current_position + A * t1 * (t - t1/2);
			qt_dot = A * t1;
		}
		else if ( t > (t_total - t1) && t <= t_total)
		{
			qt = Target_pos - 0.5 * A * (t - t_total) * (t - t_total);
			qt_dot = A * (t_total - t );
			
		}
		
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == TIM2)
	{
    
		////////////////////////////////////////// Encoder A ////////////////////////////////////////////////////
		EncoderA = __HAL_TIM_GetCounter(&htim3);
		//Lay chenh lech encoder
		deltaA = abs(EncoderA - EncoderA_pre);
		//Neu gia tri chenh lech lon hon gia tri xung/vong la vo ly, nen lay gia tri tran tru cho chenh lech
		if (deltaA >= __HAL_TIM_GET_AUTORELOAD(&htim3) / 2)
			{
				deltaA = __HAL_TIM_GET_AUTORELOAD(&htim3) - deltaA;
			}
		
		CurvelA = ( 60 * deltaA  ) / ( PPR * Time_Interrupt*gear_ratio);
		WR = Average_5_times(CurvelA,sum_A);	
		////////////////////////////////////////// Encoder B /////////////////////////////////////////////////////	
		EncoderB = __HAL_TIM_GetCounter(&htim4);
		//Lay chenh lech encoder
		deltaB = abs(EncoderB - EncoderB_pre);
		//Neu gia tri chenh lech lon hon gia tri xung/vong la vo ly, nen lay gia tri tran tru cho chenh lech
		if (deltaB  >= __HAL_TIM_GET_AUTORELOAD(&htim4) / 2)
		{
			deltaB = __HAL_TIM_GET_AUTORELOAD(&htim4) - deltaB;
		}
		
		CurvelB = ( 60 * deltaB ) / (PPR * Time_Interrupt *gear_ratio);
		WL = Average_5_times(CurvelB,sum_B);
		
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		TempPWM_L = PID_WL(SP_WL,WL);
    TempPWM_R = PID_WR(SP_WR,WR);

    OutPWM_L = TempPWM_L + Anti_windup(TempPWM_L,2.375,999,0);
    OutPWM_R = TempPWM_R + Anti_windup(TempPWM_R,2.1875,999,0);

    U_out_L = round(OutPWM_L);
    U_out_R = round(OutPWM_R);


	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,U_out_R);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,U_out_L);
		
		EncoderA_pre = EncoderA;
		EncoderB_pre = EncoderB;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		char s[] = "x: 10, y: 20";
    int cur_x, cur_y;
    get_position(s, &cur_x, &cur_y);
	
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1 | TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3,GPIO_PIN_SET);
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
  htim1.Init.Prescaler = 3;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
