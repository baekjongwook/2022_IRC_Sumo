/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
#include "pid_control_long.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//TIMER
#define DEGREE 22

#define BOOST 950
#define SECOND 110
//1000 - 95
//950 - 120 -> 105
//850 - 145 -> 130
//800 - 170

#define PSD_90 37;
#define PSD_STOP 300;
#define PSD_GO 10;
#define PSD_TURN 5;
#define PSD_TURN2 10;

//PSD
#define PSD_HIT 1000
#define PSD_BOOST 2200

#define PSD_RUN 1650

//VELOCITY
#define FRONT 500
#define FRONT_BOOST 700
#define BACK 800
#define TURN 550

//JUDGE
#define IR_JUDGE 1300

#define UP_PAN_MAX 3500
#define UP_PAN_MIN 800
#define UP_NO_PAN 500

#define DOWN_PAN_MAX 3500
#define DOWN_PAN_MIN 900
#define DOWN_NO_PAN 800

//EMERGENCY
#define EMERGENCY 1500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//ADC
int ADC_Buffer[16] = {0, };
int Real_ADC_Buffer[7] = {0, };

int ADC_0[5] = {0, }; //IR1
int ADC_1[5] = {0, }; //IR2
int ADC_2[5] = {0, }; //PSD1
int ADC_3[5] = {0, }; //PSD2
int ADC_4[5] = {0, }; //PSD3
int ADC_5[5] = {0, }; //PSD4
int ADC_6[5] = {0, }; //PSD5

int downcam = 0; //CAM1
int upcam = 0; //CAM2

int presentback1 =  0;
int presentback2 =  0;
int pastback1 =  0;
int pastback2 =  0;
int minusback1 = 0;
int minusback2 = 0;

int count = 0;

//motor
long long now_L = 0;
long long now_R = 0;
long long past_L = 0;
long long past_R = 0;
long long m1_L = 0;
long long m1_R = 0;
float RPM_L = 0;
float RPM_R = 0;
float Degree_L = 0;
float Degree_R = 0;

//PID
LPID leftmotor;
LPID rightmotor;
LPID leftmotor_Degree;
LPID rightmotor_Degree;

int frontirflag = 0;
int cameraflag = 0;
int hitflag = 0;
int startflag = 0;
int psdflag = 0;
int a = 0;

//timer
int hitcount = 0;

int startcount = 0;

int firstleft = 0;
int secondfront = 0;
int thirdleft = 0;
int fourthfront = 0;

int PSD_1 = 0;
int PSD_2 = 0;
int PSD_3 = 0;
int PSD_4 = 0;
int PSD_5 = 0;
int PSD_6 = 0;
int PSD_7 = 0;

//mode
int controlmode = 0;

void ADCFilter()
{
	ADC_0[count] = ADC_Buffer[0];
	ADC_1[count] = ADC_Buffer[1];
	ADC_2[count] = ADC_Buffer[2];
	ADC_3[count] = ADC_Buffer[3];
	ADC_4[count] = ADC_Buffer[4];
	ADC_5[count] = ADC_Buffer[5];
	ADC_6[count] = ADC_Buffer[8];

	downcam = ADC_Buffer[6];
	upcam = ADC_Buffer[7];

	count++;

	Real_ADC_Buffer[0] = (ADC_0[1] + ADC_0[2] + ADC_0[3] + ADC_0[4] + ADC_0[0]) / 5;
	Real_ADC_Buffer[1] = (ADC_1[1] + ADC_1[2] + ADC_1[3] + ADC_1[4] + ADC_1[0]) / 5;
	Real_ADC_Buffer[2] = (ADC_2[1] + ADC_2[2] + ADC_2[3] + ADC_2[4] + ADC_2[0]) / 5;
	Real_ADC_Buffer[3] = (ADC_3[1] + ADC_3[2] + ADC_3[3] + ADC_3[4] + ADC_3[0]) / 5;
	Real_ADC_Buffer[4] = (ADC_4[1] + ADC_4[2] + ADC_4[3] + ADC_4[4] + ADC_4[0]) / 5;
	Real_ADC_Buffer[5] = (ADC_5[1] + ADC_5[2] + ADC_5[3] + ADC_5[4] + ADC_5[0]) / 5;
	Real_ADC_Buffer[6] = (ADC_6[1] + ADC_6[2] + ADC_6[3] + ADC_6[4] + ADC_6[0]) / 5;

	if(count == 5)
		count = 0;
}

void GetRPM_L()
{
	past_L = now_L;
	now_L = TIM4->CNT;
	m1_L = now_L - past_L;

	if(m1_L < -60000)
	{
		m1_L += 65535;
	}
	else if(m1_L > 60000)
	{
		m1_L -= 65535;
	}

	RPM_L = (60.0 * m1_L) / (0.01 * 2048.0);

	Degree_L += (m1_L / (8192.0 / 360.0));
}

void GetRPM_R()
{
	past_R = now_R;
	now_R = TIM3->CNT;
	m1_R = now_R - past_R;

	if(m1_R < -60000)
	{
		m1_R += 65535;
	}
	else if(m1_R > 60000)
	{
		m1_R -= 65535;
	}

	RPM_R = (60.0 * m1_R) / (0.01 * 2048.0) * (-1);

	Degree_R += (m1_R / (8192.0 / 360.0)) * (-1);
}

void PID_Velocity(float *input_L, float *input_R)
{
	PID_Control_Long(&leftmotor, leftmotor.target, *input_L);
	PID_Control_Long(&rightmotor, rightmotor.target, *input_R);

	if(leftmotor.nowOutput < 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		TIM1->CCR2 = leftmotor.nowOutput * (-1);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		TIM1->CCR2 = leftmotor.nowOutput;
	}

	if(rightmotor.nowOutput < 0)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		TIM1->CCR1 = rightmotor.nowOutput * (-1);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		TIM1->CCR1 = rightmotor.nowOutput;
	}
}

void PID_Velocity_Position(float *input_L, float *input_R)
{
	PID_Control_Long(&leftmotor_Degree, leftmotor_Degree.target, *input_L);
	PID_Control_Long(&rightmotor_Degree, rightmotor_Degree.target, *input_R);

	leftmotor.target = leftmotor_Degree.nowOutput * (-1);
	rightmotor.target = rightmotor_Degree.nowOutput;
}

void SetPID_Speed()
{
	leftmotor.outputLimit = 200;
	leftmotor.errorSumLimit = 5000;
	leftmotor.underOfPoint = 1000;
	leftmotor.kP = 3800;
	leftmotor.kI = 125;
	leftmotor.kD = 50;

	rightmotor.outputLimit = 900;
	rightmotor.errorSumLimit = 5000;
	rightmotor.underOfPoint = 1000;
	rightmotor.kP = 4000;
	rightmotor.kI = 140;
	rightmotor.kD = 0;
}

void SetPID_Position()
{
  	leftmotor.outputLimit = 900; //Duty : 90%(900)
	leftmotor.errorSumLimit = 5000;
	leftmotor.underOfPoint = 1000;
	leftmotor.kP = 0;
	leftmotor.kI = 0;
	leftmotor.kD = 0;

	rightmotor.outputLimit = 900; //900 //Duty : 90%(900)
	rightmotor.errorSumLimit = 5000;
	rightmotor.underOfPoint = 1000;
	rightmotor.kP = 0;
	rightmotor.kI = 0;
	rightmotor.kD = 0;
}

void PID_Start()
{
	GetRPM_L();
	GetRPM_R();

	if(controlmode == 2)
	{
		SetPID_Position();
	}

	PID_Velocity(&RPM_L, &RPM_R);
}

void JudgeFrontIR()
{
	frontirflag = !(Real_ADC_Buffer[0] > IR_JUDGE && Real_ADC_Buffer[1] > IR_JUDGE) ? 1 : 0;
}

void JudgeCamera() //flag : 1 = front, 2 = left, 3 = right, 4 = fail
{
	if(upcam > UP_PAN_MIN && upcam <= UP_PAN_MAX)
	{
		cameraflag = 1;
	}

	else if(upcam > UP_PAN_MAX)
	{
		cameraflag = 2;
	}

	else if(upcam > UP_NO_PAN && upcam <= UP_PAN_MIN)
	{
		cameraflag = 3;
	}


	else if(downcam > DOWN_PAN_MIN && downcam <= DOWN_PAN_MAX)
	{
		cameraflag = 1;
	}

	else if(downcam > DOWN_PAN_MAX)
	{
		cameraflag = 2;
	}

	else if(downcam > DOWN_NO_PAN && downcam <= DOWN_PAN_MIN)
	{
		cameraflag = 3;
	}


	else
	{
		cameraflag = 0;
	}
}

void Stop()
{
	controlmode = 0;

	leftmotor.target = 0;
    rightmotor.target = 0;
}

void GoFront()
{
	controlmode = 0;

	leftmotor.target = FRONT;
    rightmotor.target = FRONT;
}

void GoFrontBoost()
{
	controlmode = 0;

	leftmotor.target = FRONT_BOOST;
	rightmotor.target = FRONT_BOOST;
}

void GoBack()
{
	controlmode = 0;

	leftmotor.target = -BACK;
	rightmotor.target = -BACK;
}

void TurnLeft()
{
	controlmode = 0;

	leftmotor.target = -TURN;
	rightmotor.target = TURN;
}

void TurnRight()
{
	controlmode = 0;

	leftmotor.target = TURN;
	rightmotor.target = -TURN;
}

void PWM_Stop()
{
	controlmode = 1;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	TIM1->CCR2 = 0;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	TIM1->CCR1 = 0;
}

void PWM_GoFront()
{
	controlmode = 1;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	TIM1->CCR2 = 600;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	TIM1->CCR1 = 600;
}

void PWM_GoFrontBoost()
{
	controlmode = 1;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	TIM1->CCR2 = BOOST;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	TIM1->CCR1 = BOOST;
}

void PWM_GoBack()
{
	controlmode = 1;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	TIM1->CCR2 = 900;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	TIM1->CCR1 = 900;
}

void PWM_TurnLeft()
{
	controlmode = 1;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	TIM1->CCR2 = 650;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	TIM1->CCR1 = 650;
}

void PWM_TurnRight()
{
	controlmode = 1;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	TIM1->CCR2 = 650;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	TIM1->CCR1 = 650;
}

void PWM_TurnBackRight()
{
	controlmode = 1;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	TIM1->CCR2 = 650;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	TIM1->CCR1 = 650;
}

void HitFloor()
{
	hitflag == 1 ? PWM_GoBack() : PWM_GoFrontBoost();
}

void Algorithm()
{
	if(frontirflag == 1)
	{
		if(Real_ADC_Buffer[4] > EMERGENCY && Real_ADC_Buffer[5] > EMERGENCY)
		{
			PWM_GoFrontBoost();
		}

		else
		{
			GoBack();
		}
	}

	else if((Real_ADC_Buffer[2] > PSD_BOOST || Real_ADC_Buffer[3] > PSD_BOOST) && !(cameraflag == 0))
	{
		if(Real_ADC_Buffer[6] < PSD_RUN)
		{
			PWM_GoBack();
		}

		else
		{
			PWM_GoFrontBoost();
		}
	}

	else if(((Real_ADC_Buffer[2] > PSD_HIT && Real_ADC_Buffer[3] > PSD_HIT) && (Real_ADC_Buffer[2] <= PSD_BOOST && Real_ADC_Buffer[3] <= PSD_BOOST)) && !(cameraflag == 0))
	{
		HitFloor();
	}

	else
	{
		switch(cameraflag)
		{
		case 1:
		{
			GoFrontBoost();

			break;
		}

		case 2:
		{
			TurnLeft();

			break;
		}

		case 3:
		{
			TurnRight();

			break;
		}

		default:
		{
			TurnLeft();

			break;
		}
		}
	}
}

void Start_Algorithm()
{
	JudgeFrontIR();
	JudgeCamera();

	if(startflag == 1)
	{
		Algorithm();
	}

	else if(startcount > fourthfront)
	{
		startflag = 1;
	}

	else if(startcount > thirdleft && startcount <= fourthfront)
	{
		startcount++;

		HitFloor();
	}

	else if(startcount > secondfront && startcount <= thirdleft)
	{
		startcount++;

		TurnLeft();
	}

	else if(startcount > firstleft && startcount <= secondfront)
	{
		startcount++;

		PWM_GoFrontBoost();
	}

	else
	{
		startcount++;

		TurnLeft();
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

	leftmotor.outputLimit = 900;
	leftmotor.errorSumLimit = 5000;
	leftmotor.underOfPoint = 1000;
	leftmotor.kP = 500;
	leftmotor.kI = 100;
	leftmotor.kD = 0;

	rightmotor.outputLimit = 900;
	rightmotor.errorSumLimit = 5000;
	rightmotor.underOfPoint = 1000;
	rightmotor.kP = 500;
	rightmotor.kI = 100;
	rightmotor.kD = 0;

	leftmotor_Degree.outputLimit = 350; //Duty : 90%(900)
	leftmotor_Degree.errorSumLimit = 5000;
	leftmotor_Degree.underOfPoint = 1000;
	leftmotor_Degree.kP = 0;
	leftmotor_Degree.kI = 0;
	leftmotor_Degree.kD = 0;

	rightmotor_Degree.outputLimit = 350; //900 //Duty : 90%(900)
	rightmotor_Degree.errorSumLimit = 5000;
	rightmotor_Degree.underOfPoint = 1000;
	rightmotor_Degree.kP = 0;
	rightmotor_Degree.kI = 0;
	rightmotor_Degree.kD = 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  firstleft = DEGREE;
  secondfront = DEGREE + SECOND;
  thirdleft = secondfront + (DEGREE * 3);
  fourthfront = thirdleft + (SECOND / 2);

  PSD_1 = PSD_90;
  PSD_2 = PSD_1 + PSD_STOP;
  PSD_3 = PSD_2 + PSD_GO;
  PSD_4 = PSD_3 + PSD_TURN;
  PSD_5 = PSD_4 + PSD_STOP;
  PSD_6 = PSD_5 + PSD_GO;
  PSD_7 = PSD_6 + PSD_TURN2;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

	//timer6 - 5ms period
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer, 16);

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim8); //ADC Timer

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //R
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //L

  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM8)
	{
		ADCFilter();
	}

	if(htim->Instance == TIM6) //10ms
	{
		if(controlmode == 0)
		{
			PID_Start();
		}

		Start_Algorithm();
	}

	if(htim->Instance == TIM7) //20ms(0.02s)
	{
		if(controlmode == 2)
		{
			PID_Velocity_Position(&Degree_L, &Degree_R);
		}

		hitcount++;

		if(hitcount == 5)
		{
			hitflag = hitflag == 0 ? 1 : 0;

			hitcount = 0;
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
