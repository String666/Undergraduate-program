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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t ucLed;

u8 ucState=0;
u8 ucPlat = 1, ucSet=0;
u8 ucArrive=0;
u32 ulKey_Ms;

vu32 Capture = 0; 
vu32  Duty=0;
float Freq=0;


__IO uint32_t ulTickms=0;
vu32 ulSec=0;
vu32 ul250Ms=0;
u8 ucHour=0, ucMin=0, ucSec=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ELEV_Proc(void);
u8 Key_Proc(void);
void Lcd_Proc();
void Led_Proc(void);

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
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Blue);
	LCD_SetBackColor(Blue);
	LCD_SetTextColor(White);
	
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2,TIM_CHANNEL_2);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		ELEV_Proc();
		Lcd_Proc();		
		Led_Proc();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ELEV_Proc(void)
{
	static u32 ulElevMs;
	u8 ucRst;
	switch(ucState)
	{
		case 0:			//set
			ucRst=Key_Proc();
			if(ucRst)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				htim3.Instance->ARR=499;
				htim3.Instance->CCR2=250;
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
				ucState=4;
				ulElevMs=ulTickms;
			}	
			break;
		case 1:					//up			6s
			if(ulTickms-ulElevMs>=6000)
			{
				if(ucSet & 1<<(ucPlat-1))			//tartger floor
				{
					ucSet &=~(1<<(ucPlat-1));
					ucLed &=~(1<<(ucPlat-1));					
					
					HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
					ucArrive=1;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
					htim3.Instance->ARR=499;
					htim3.Instance->CCR2=300;
					HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
			
					ucState=3;									
					ulElevMs=ulTickms;
				}
				else
				{
					ucPlat=ucPlat+1;
					ulElevMs=ulTickms;		//continue up
				}
			}
			break;
		case 2:					//down		6s
			if(ulTickms-ulElevMs>=6000)
			{
				if(ucSet & 1<<(ucPlat-1))			//tartger floor
				{
					ucSet &=~(1<<(ucPlat-1));
					ucLed &=~(1<<(ucPlat-1));
					
					HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
					ucArrive=1;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
					htim3.Instance->ARR=499;
					htim3.Instance->CCR2=300;
					HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
							
					ucState=3;					
					ulElevMs=ulTickms;
				}
				else
				{
					ucPlat=ucPlat-1;
					ulElevMs=ulTickms;		//continue up
				}
			}
			break;
		case 3:					//open door   4s
			if(ulTickms-ulElevMs>=4000)
			{
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
				if(ucSet)			//  -->wait 2s
				{
					ulElevMs=ulTickms;
					ucState=5;					
				}					
				else //  -->set
				{
					ucState=0;
				}
				
			}
			break;
		case 4:					//close door  4s    
			if(ulTickms-ulElevMs>=4000)
			{
				HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
				if((ucSet & ~((1<<ucPlat) -1) )>0)				// -->up
				{
					ucPlat++;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);	//ELEV up
					htim3.Instance->ARR=999;									//2KHz
					htim3.Instance->CCR1=800;									//80%
					HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
					
					ucState=1;
					ulElevMs=ulTickms;
				}
				else				//--<down
				{
					ucPlat--;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);	//ELEV down
					htim3.Instance->ARR=999;									//2KHz
					htim3.Instance->CCR1=600;									//60%
					HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
					
					ucState=2;
					ulElevMs=ulTickms;
				}				
			}
			break;
		case 5:					//wait 2s
			if(ulTickms-ulElevMs>=2000)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				htim3.Instance->ARR=499;				//2K
				htim3.Instance->CCR2=250;				//50%
				HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
				
				ucState=4;
				ulElevMs=ulTickms;
			}
			break;
		
	}
}

void Led_Proc(void)
{
	static u8 temp1=4;   //>>  up
	static u8 temp2=0;		//<<down
	static u32 ulSec1=0;
	if(ulSec1!=ulSec)
	{
		ulSec1=ulSec;
		switch(ucState)
		{
			case 1:
				ucLed &=0x0F;
				ucLed |=0x80>>temp2;
				if(++temp2==4) temp2=0;
				break;
			case 2:
				ucLed &=0x0F;
				ucLed |=1<<temp1;
				if(++temp1==8) temp1=4;
				break;
			default:
				ucLed &=0x0F;
				break;		
		}
	}
	LED_Disp(ucLed);
}


u8 Key_Proc(void)
{
	u8 ucKey_Val;
	static u8 ucKey_Old;
	//static u32 ulKey_Ms;
	ucKey_Val =KEY_Read();
	if(ucKey_Val!=ucKey_Old)
	{
		ucKey_Old=ucKey_Val;
		ulKey_Ms=ulTickms;
		if(ucKey_Val!=ucPlat)
		{
			switch(ucKey_Val)
			{
				case 1:
						ucSet |=1<<0;
						ucLed |=1<<0;						
						break;
				case 2:
						ucSet |=1<<1;
						ucLed |=1<<1;
						break;
				case 3:
						ucSet |=1<<2;
						ucLed |=1<<2;
						break;
				case 4:
						ucSet |=1<<3;
						ucLed |=1<<3;
						break;				
			}	
			LED_Disp(ucLed);
				
		}
	}
	if(ucKey_Old==0 && ulTickms-ulKey_Ms>=1000 && ucSet>0)
	{
		return 1;
	}
	else
		return 0;
}

void Lcd_Proc()
{
	static u8 Num=0;
	static u32 Temp_250=0;
	u8 pucDisp[25]={0};	
	u8 *state_str[]={"Stopping","Elev Up","Elev Down","Door Open","Door Close","Wait 2s"};
	LCD_DisplayStringLine(Line0,(unsigned char *)"  Current Platform  ");
	
	if(ucArrive)
	{	
		if(Temp_250!=ul250Ms)
		{
			Temp_250=ul250Ms;
			if(Num++ % 2==0)
				LCD_DisplayChar(Line1,160,' ');
				//LCD_SetTextColor(White);
			else
				//LCD_SetTextColor(Red);
			LCD_DisplayChar(Line1,160,ucPlat +0x30);
		}
		if(Num==4)
		{
			ucArrive=0;
			Num=0;
			//LCD_SetTextColor(White);
		}
	}
	else
		LCD_DisplayChar(Line1,160,ucPlat +0x30);
	
	//LCD_SetTextColor(White);
	sprintf(pucDisp,"      %02d:%02d:%02d",ucHour,ucMin,ucSec);
	LCD_DisplayStringLine(Line2,pucDisp);
	sprintf(pucDisp,"      %s    ",state_str[ucState]);
	LCD_DisplayStringLine(Line3,pucDisp);
	//LCD_DisplayChar(Line8,160,ucState  +0x30);	
	sprintf(pucDisp,"  Freq:%4.1fHZ      ",Freq);
	LCD_DisplayStringLine(Line6,(unsigned char *)pucDisp);
	sprintf(pucDisp,"  Duty:%4.1f%%      ",100.0*Duty/Capture);
	LCD_DisplayStringLine(Line7,(unsigned char *)pucDisp);
	sprintf(pucDisp,"  Capt1:%d      ",Capture);
	LCD_DisplayStringLine(Line8,(unsigned char *)pucDisp);
	sprintf(pucDisp,"  Duty1:%d      ",Duty);
	LCD_DisplayStringLine(Line9,(unsigned char *)pucDisp);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance ==TIM2)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
		{
			Capture=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1)+1;		
			if(Capture !=0)
			{
				Freq=HAL_RCC_GetSysClockFreq()/80.0/Capture;
				Duty=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2)+1;
			}
			else
			{
				Freq=0;
				Duty=0;
			}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
