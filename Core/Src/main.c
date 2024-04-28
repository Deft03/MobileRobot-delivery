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
#include "string.h"
#include "stdio.h" 
#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi 3.14159265359
#define PPR 1000
#define Time_Interrupt 0.01
#define High_limit_PWM 99
#define Low_Limit_PWM 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int32_t EncoderA, EncoderA_pre, EncoderB, EncoderB_pre;
int32_t  CountA, CountB;
uint8_t PreviousState;
float CurvelR,CurvelL;
double WL,WR;
float sumA[20],sumB[20];
uint8_t msg[30];
float pwm;

//PID
struct PID
{
  float Kp, Ki, Kd;
  float Up, Up_1;
  float Ui, Ui_1;
  float Ud, Ud_1;
  double Ui_Anti;
  uint16_t Out_PWM;
  double Temp_PWM;
}PID_WL, PID_WR;

//Error PID
struct error_PID
{
  double ek, ek_1;
}Error_WR,Error_WL;

//pointer
struct PID* PID_wl = &PID_WL;
struct PID* PID_wr = &PID_WR;
struct error_PID* Error_wl= &Error_WL;
struct error_PID* Error_wr= &Error_WR;

//Setpoint
float SP_WL, SP_WR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///////////////////////Read Encoder A///////////////////////////////////////////
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
		unsigned char state0;
	state0 = (state0<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	state0 = (state0<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	state0 = state0&0x03;
	switch(state0) {
		case 0:
			if(PreviousState==1) EncoderA++;
			else EncoderA--;
		break;
		case 1:
			if(PreviousState==3) EncoderA++;
			else EncoderA--;
		break;
		case 2:
			if(PreviousState==0) EncoderA++;
			else EncoderA--;
		break;
		case 3:
			if(PreviousState==2) EncoderA++;
			else EncoderA--;
		break;
		}
	
	PreviousState=state0;
	
	if (EncoderA>=PPR || EncoderA <= -PPR)
  {
		EncoderA=0;
	}
	
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
unsigned char state1;
	state1 = (state1<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	state1 = (state1<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	state1 = state1&0x03;
	switch(state1) {
		case 0:
			if(PreviousState==1) EncoderA++;
			else EncoderA--;
		break;
		case 1:
			if(PreviousState==3) EncoderA++;
			else EncoderA--;
		break;
		case 2:
			if(PreviousState==0) EncoderA++;
			else EncoderA--;
		break;
		case 3:
			if(PreviousState==2) EncoderA++;
			else EncoderA--;
		break;
		}
	PreviousState=state1;
	
	if (EncoderA>=PPR)
  {
    EncoderA=0;
  }
	if (EncoderA<=-PPR)
  {
    EncoderA=0;
  }
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
		
  /* USER CODE END EXTI1_IRQn 1 */
}

/////////////////////////////////////Read Encoder B //////////////////////////////////////////

void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	unsigned char state2;
	state2 = (state2<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	state2 = (state2<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	state2 = state2&0x03;
	switch(state2) {
		case 0:
			if(PreviousState==1) EncoderB++;
			else EncoderB--;
		break;
		case 1:
			if(PreviousState==3) EncoderB++;
			else EncoderB--;
		break;
		case 2:
			if(PreviousState==0) EncoderB++;
			else EncoderB--;
		break;
		case 3:
			if(PreviousState==2) EncoderB++;
			else EncoderB--;
		break;
		}
	PreviousState=state2;
	
	if (EncoderB>=1000)EncoderB=0;
	if (EncoderB<=-1000)EncoderB=0;
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	unsigned char state3;
	state3 = (state3<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	state3 = (state3<<1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	state3 = state3&0x03;
	switch(state3) {
		case 0:
			if(PreviousState==1) EncoderB++;
			else EncoderB--;
		break;
		case 1:
			if(PreviousState==3) EncoderB++;
			else EncoderB--;
		break;
		case 2:
			if(PreviousState==0) EncoderB++;
			else EncoderB--;
		break;
		case 3:
			if(PreviousState==2) EncoderB++;
			else EncoderB--;
		break;
		}
	PreviousState=state3;
	
	if (EncoderB>=1000)EncoderB=0;
	if (EncoderB<=-1000)EncoderB=0;
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}



float Average_5_time(float Var, float Temp[20]){
	float sum = 0, out_average_var;// Initialize sum to 0
	for (int i =0; i<19;i++)
	{
		Temp[i] = Temp[i+1];//gan gia tri hien tai vao gia tri truoc
		sum += Temp[i];//cong dan cac gia tri vua luu
	}
	//gan gia tri moi nhat
	Temp[19]=Var;
	sum += Temp[19];
	//tinh trung binh
	out_average_var = sum/20;
	return out_average_var;
}

float Anti_Windup(float Out_PWM, uint16_t HIGH_Limit, uint16_t LOW_Limit, float Kb)
{
    float e_reset = 0;
    float Ui_anti;

    if (Out_PWM > HIGH_Limit)
    {
        e_reset = (HIGH_Limit - Out_PWM );
    }
    else if (Out_PWM < LOW_Limit)
    {
        e_reset = (LOW_Limit - Out_PWM);
    }
    else
    {
        e_reset = 0;
    }
        Ui_anti = Time_Interrupt * e_reset * Kb;


    return Ui_anti;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
	if (htim->Instance==TIM3)
  {
    //////////////////////////////////////ENCODER A /////////////////////////////////////////////////
    //Lấy chênh lệch Encoder
		CountA = abs(EncoderA-EncoderA_pre);
		//Xử lý tràn
		if(CountA >= 500)
    {
     CountA = 1000 - CountA;
    }
		//Tính giá trị rad/s hiện tại
		CurvelR = (CountA * 2 * pi) / (PPR * Time_Interrupt);
		//Tính trung bình
		WR = Average_5_time(CurvelR,sumA);
		
    ////////////////////////////////////ENCODER B ///////////////////////////////////////////////////
    //Lấy chênh lệch Encoder
    CountB = abs(EncoderB-EncoderB_pre);
    //Xử lý tràn
    if(CountB >= 500)
    {
      CountB = 1000 - CountB;
    }
    //Tính giá trị rad/s hiện tại
    CurvelL = (CountB * 2 * pi) / (PPR * Time_Interrupt);
    //Tính trung bình
    WL = Average_5_time(CurvelL,sumB);
		
    //PID controller
    Error_wl -> ek = SP_WL - WL;
    Error_wr -> ek = SP_WR - WR;

    // Kenh A
		PID_wl -> Up =  PID_wl -> Kp * Error_wl -> ek;
		PID_wl -> Ui = PID_wl -> Ui_1 + PID_wl -> Ki * Error_wl -> ek * Time_Interrupt + PID_wl -> Ui_Anti;
		PID_wl -> Ud = PID_wl -> Ud_1 * (Error_wl -> ek - Error_wl -> ek_1)/Time_Interrupt;

    //Antiwindup
    PID_wl -> Ui_Anti = Anti_Windup(PID_wl -> Temp_PWM, High_limit_PWM, Low_Limit_PWM, Kb);
    PID_wl -> Temp_PWM = PID_wl -> Up + PID_wl -> Ui + PID_wl -> Ud;

    PID_wl -> Out_PWM = round( PID_wl -> Temp_PWM);

    if(PID_wl -> Out_PWM >= 999)
		{
			PID_wl -> Out_PWM = 999;
		}
		else if (PID_wl -> Out_PWM <=0 )
		{
			PID_wl -> Out_PWM = 0;
		}

    //Kenh B
    PID_wr -> Up =  PID_wr -> Kp * Error_wr -> ek;
		PID_wr -> Ui = PID_wr -> Ui_1 + PID_wr -> Ki * Error_wr -> ek * Time_Interrupt + PID_wr -> Ui_Anti;
		PID_wr -> Ud = PID_wr -> Ud_1 * (Error_wr -> ek - Error_wr -> ek_1)/Time_Interrupt;

    //Antiwindup
    PID_wr -> Ui_Anti = Anti_Windup(PID_wr -> Temp_PWM, High_limit_PWM, Low_Limit_PWM, Kb);
    PID_wr -> Temp_PWM = PID_wr -> Up + PID_wr -> Ui + PID_wr -> Ud;

    PID_wr -> Out_PWM = round( PID_wr -> Temp_PWM);

    if(PID_wr -> Out_PWM >= 999)
		{
			PID_wr -> Out_PWM = 999;
		}
		else if (PID_wr -> Out_PWM <=0 )
		{
			PID_wr -> Out_PWM = 0;
		}

    EncoderA_pre = EncoderA;
    EncoderB_pre = EncoderB;

    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,PID_wl->Out_PWM);
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,PID_wr->Out_PWM);  
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
  // Gan gia tri PID
  //DC1
  PID_wl -> Kp = 0.42247;
  PID_wl -> Ki = 0.14922;
  PID_wl -> Kd = 0;
  //DC2
  PID_wr -> Kp = 0.39474;
  PID_wr -> Ki = 0.13945;
  PID_wr -> Kd = 0;
  //Gan gia tri ban dau
  PID_wl -> Ui_1 = 0;
  PID_wr -> Ui_1 = 0;
  PID_wl -> Ud_1 = 0;
  PID_wr -> Ud_1 = 0;
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
  MX_USART1_UART_Init();
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
		sprintf((char*)msg,"%f \n\r",WL);
		HAL_UART_Transmit(&huart1,msg,sizeof(msg),100);
		HAL_Delay(10);
		
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
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
  huart1.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
