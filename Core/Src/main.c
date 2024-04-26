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

  * Control 1 DC Motor Servo 
  * @author         : TRAN THANH SANG
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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Pi 3.1415926535897932384626433832795
#define error -0.0088              //error when rotate (motor) [NEEDED EDITING]
#define Kp_R 26.5258238789594       //[PID Parameter](100/3.76991118);//kp=in/out=100/36
#define Ki_R 0.005                  //PID Parameter
#define Kp_L 26.5258238789594       //[PID Parameter](100/3.76991118);//kp=in/out=100/36
#define Ki_L 0.005                  //PID Parameter
#define PULSES 10752              //Read ENCODER X4
#define SAMPLETIME  0.005         //Interrupt TIMER 4 5ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//
/* ------------------MOTOR RIGHT ---------------------------------------------------------*/

/*========== ENCODER INTERRUPT EXTI 4 =====================*/
uint8_t PreviousState_R;            // Save State Interrupt
int16_t CountValue_R;               // Counter Pulses from Encoder
uint16_t CntVel_R;                  // VEL Counter
float PulseToRad = PULSES / (2*Pi);

/*========== INTERRUPT TIMER 4 CALCULATE VEL VARIABLES ============*/
float CurrentSpeed_R;               // Current Speed (rad/s)
float DesiredSpeed_R;               // Desired Speed (rad/s)
float RealSpeed_R;                  // Current Speed (RPM)
int16_t Cnttmp_R;

/*=========== PID VARIABLES ===============================================*/
uint8_t HILIM_R;                    // Limit PWM HIGH
uint8_t LOLIM_R;                    // Limit PWM LOW     
uint8_t PWM_R;                      // Pulse PWM 0 -> 100 PWM
float uout_R;                       // [PID] Value return
float err_R;                        // [PID] error

/*========== TRANSFER DATA USING UART ============*/
char buffer[32]={0};

/* -------------------------------------------------------------------------------------------------*/

/* ------------------MOTOR LEFT ---------------------------------------------------------*/

/*========== ENCODER INTERRUPT EXTI 4 =====================*/
uint8_t PreviousState_L;            // Save State Interrupt
int16_t CountValue_L;               // Counter Pulses from Encoder
uint16_t CntVel_L;                  // VEL Counter
float PulseToRad_L = PULSES / (2*Pi);

/*========== INTERRUPT TIMER 4 CALCULATE VEL VARIABLES ============*/
float CurrentSpeed_L;               // Current Speed (rad/s)
float DesiredSpeed_L;               // Desired Speed (rad/s)
float RealSpeed_L;                  // Current Speed (RPM)
int16_t Cnttmp_L;

/*=========== PID VARIABLES ===============================================*/
uint8_t HILIM_L;                    // Limit PWM HIGH
uint8_t LOLIM_L;                    // Limit PWM LOW     
uint8_t PWM_L;                      // Pulse PWM 0 -> 100 PWM
float uout_L;                       // [PID] Value return
float err_L;                        // [PID] error

/*========== TRANSFER DATA USING UART ============*/
char buffer[32]={0};

/* -------------------------------------------------------------------------------------------------*/



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 PUTCHAR_PROTOTYPE 
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
 HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
 return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uprintf(char *str){
  HAL_UART_Transmit(&huart1,(uint8_t *)str,sizeof(str), 100);

}
//============= Encoder for motor RIGHT==========

void EXTI9_5_IRQHandler(void){	// doc encoder	
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
unsigned char State0;
	State0 = (State0<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	State0 = (State0<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	State0 = State0&0x03;
	switch (State0) {
		case 0:
			if(PreviousState_R==1) CountValue_R++;
			else CountValue_R--;
		break;
		case 1:
			if(PreviousState_R==3) CountValue_R++;
			else CountValue_R--;
		break;
		case 2:
			if(PreviousState_R==0) CountValue_R++;
			else CountValue_R--;
		break;
		case 3:
			if(PreviousState_R==2) CountValue_R++;
			else CountValue_R--;
		break;
		}
	PreviousState_R = State0;
	CntVel_R++;
	if (CountValue_R>=PULSES) {
		CountValue_R = 0;
	}
	else if	(CountValue_R<=-PULSES) {
		CountValue_R = 0;
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
}

void EXTI4_IRQHandler(void){	// doc encoder	
  /* USER CODE BEGIN EXTI4_IRQn 0 */
unsigned char State1;
	State1 = (State1<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	State1 = (State1<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	State1 = State1&0x03;
	switch (State1) {
		case 0:
			if(PreviousState_R==1) CountValue_R++;
			else CountValue_R--;
		break;
		case 1:
			if(PreviousState_R==3) CountValue_R++;
			else CountValue_R--;
		break;
		case 2:
			if(PreviousState_R==0) CountValue_R++;
			else CountValue_R--;
		break;
		case 3:
			if(PreviousState_R==2) CountValue_R++;
			else CountValue_R--;
		break;
		}
	PreviousState_R = State1;
	CntVel_R++;
	if (CountValue_R>=PULSES) {
		CountValue_R = 0;
	}
	else if	(CountValue_R<=-PULSES) {
		CountValue_R = 0;
	}
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}
////////////////////////////////

//============= Encoder for motor LEFT==========

void EXTI9_5_IRQHandler(void){	// doc encoder	
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
unsigned char State0;
	State0 = (State0<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	State0 = (State0<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	State0 = State0&0x03;
	switch (State0) {
		case 0:
			if(PreviousState_L) CountValue_L++;
			else CountValue_L--;
		break;
		case 1:
			if(PreviousState_L==3) CountValue_L++;
			else CountValue_L--;
		break;
		case 2:
			if(PreviousState_L==0) CountValue_L++;
			else CountValue_L--;
		break;
		case 3:
			if(PreviousState_L==2) CountValue_L++;
			else CountValue_L--;
		break;
		}
	PreviousState_L = State0;
	CntVel_L++;
	if (CountValue_L>=PULSES) {
		CountValue_L = 0;
	}
	else if	(CountValue_L<=-PULSES) {
		CountValue_L = 0;
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
}

void EXTI4_IRQHandler(void){	// doc encoder	
  /* USER CODE BEGIN EXTI4_IRQn 0 */
unsigned char State1;
	State1 = (State1<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	State1 = (State1<<1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	State1 = State1&0x03;
	switch (State1) {
		case 0:
			if(PreviousState_L==1) CountValue_L++;
			else CountValue_L--;
		break;
		case 1:
			if(PreviousState_L==3) CountValue_L++;
			else CountValue_L--;
		break;
		case 2:
			if(PreviousState_L==0) CountValue_L++;
			else CountValue_L--;
		break;
		case 3:
			if(PreviousState_L==2) CountValue_L++;
			else CountValue_L--;
		break;
		}
	PreviousState_L = State1;
	CntVel_L++;
	if (CountValue_L>=PULSES) {
		CountValue_L = 0;
	}
	else if	(CountValue_L<=-PULSES) {
		CountValue_L = 0;
	}
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

///////////////////

//============= TIM4 for MOTOR 1 ==========
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {	// ngat timer
	if(htim->Instance==TIM4){	                                  //ngat timer 4	5ms tinh toc do
    ///////////RIGHT
    Cnttmp_R = CntVel_R;
		CntVel_R = 0;
    RealSpeed_R = (Cnttmp_R*60000)/(5*12*4*19.2);		//RPM; 
		CurrentSpeed_R = (RealSpeed_R*2*Pi)/60;  //rad/s
		//PidFunction(DesiredSpeed,RealVel);
	  PWM_R = PidFunction_R(15,CurrentSpeed_R);  //rad/s    // Run DC motor with velocity = 15 rad/s
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,PWM_R); 
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0); 

    ////////////LEFT
    Cnttmp_L = CntVel_L;
		CntVel_L = 0;
    RealSpeed_L = (Cnttmp_L*60000)/(5*12*4*19.2);		//RPM; 
		CurrentSpeed_L = (RealSpeed_L*2*Pi)/60;  //rad/s
		//PidFunction(DesiredSpeed,RealVel);
	  PWM_L = PidFunction_L(15,CurrentSpeed_L);  //rad/s    // Run DC motor with velocity = 15 rad/s
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,PWM_L); 
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,0); 

	}
}

//=============PID FUNCTION for MOTOR R==========
int PidFunction_R(float desireVel, float currentVel){
	HILIM_R = 99, LOLIM_R = 15;
	static float Ui = 0;
	float up,ui;

	err_R = desireVel-currentVel;    //ERRORs
	up = Kp_R*err_R;
	ui = Ui+Ki_R*err_R*SAMPLETIME;
	Ui = ui;
	uout_R = (int)(up+ui);
	uout_R = PWM_R+uout_R;
	
	if (uout_R > HILIM_R) uout_R = HILIM_R;
	else if( uout_R < LOLIM_R) uout_R = LOLIM_R;
	if (desireVel <= 0) uout_R = 0;
	PWM_R = uout_R;

	return PWM_R;
}
//=============PID FUNCTION for MOTOR L==========
int PidFunction_L(float desireVel, float currentVel){
	HILIM_L = 99, LOLIM_L = 15;
	static float Ui = 0;
	float up,ui;

	err_L = desireVel-currentVel;    //ERRORs
	up = Kp_L*err_L;
	ui = Ui+Ki_L*err_L*SAMPLETIME;
	Ui = ui;
	uout_L = (int)(up+ui);
	uout_L = PWM_L+uout_L;
	
	if (uout_L > HILIM_L) uout_L = HILIM_L;
	else if( uout_L < LOLIM_L) uout_L = LOLIM_L;
	if (desireVel <= 0) uout_L = 0;
	PWM_L = uout_L;

	return PWM_L;
}
//=============UART callback function==========

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1){

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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// khoi tao timer 2
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	// khoi tao timer 2
	HAL_TIM_Base_Start_IT(&htim4);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);

		sprintf(buffer,"Velocity Right: %f ---- Velocity LEFT: %f", CurVel_R, CurVel_L);
    uprintf(buffer);
    HAL_Delay(500);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 11;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 23999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
