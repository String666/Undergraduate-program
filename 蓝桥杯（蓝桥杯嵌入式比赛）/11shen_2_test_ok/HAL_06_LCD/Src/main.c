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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
//#include <stdlib.h>   malloc()
//#inclde <time.h>
#include <stdio.h>       //printf()   fputc   sprintf() sscanf()
#include "i2c.h"
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
u8 ucLed=0;
u8 ucState=0;  //0-Data
u8 ucMode=1;		//0-menu

u16 usR37;
u8 ucDuty_Auto;
u8 pucDuty_Manu[2]={10,10};

vu32 uwTickMs=0,uw100Ms=0;

u32 Capture=0;
u32 Duty=0;

u8 pucOut[20]={0};
u8 pucIn[11]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void KEY_Proc(void);
void ADC_Proc(void);
void LCD_Proc(void);
void LED_Proc(void);
void TIM_Proc(void);
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
	u8 pucI2C[2]={0};  //u8 *pucI2C[2]={0};
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
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	//HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2,TIM_CHANNEL_2);
	
	
	//sprintf(pucOut,"PA6:%02d,PA7:%02d",10,5);
	sprintf(pucOut,"%s","hello");
	//pucOut ="hello";   //  stdio.h
	//strcpy strlen strcmp    string.h
	strcpy(pucOut,"hello");
	HAL_UART_Transmit(&huart1,pucOut,5,10);
	HAL_UART_Transmit_IT(&huart1,pucOut,5);
	
	HAL_UART_Receive_IT(&huart1,pucIn,10);
	
	MEM_Read(0,pucI2C,2);
	if(pucI2C[0]>=10 && pucI2C[0]<=10 && pucI2C[0]%10==0 && pucI2C[1]>=10 && pucI2C[1]<=10 && pucI2C[1]%10==0   )
	{
		pucI2C[0]=pucDuty_Manu[0];
		pucI2C[1]=pucDuty_Manu[1];
	}
    
	//pucDuty_Manu[0],pucDuty_Manu[1]
  while (1)
  {
    ADC_Proc();
		KEY_Proc();
		LCD_Proc();
		LED_Proc();
		TIM_Proc();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void LCD_Proc(void)
{
	u8 pucDisp[25]={0};
	u8 *ppucMode[2]={"MANU","AUTO"};
	if(!ucState)
	{
		//LCD_Clear(Blue);
		LCD_DisplayStringLine(Line0,(unsigned char *)"        Data        ");
		
		sprintf(pucDisp,"  V:%4.2fV",usR37*3.3/4095);
		//sprintf(pucDisp,"  V:%-4d          ",usR37);
		LCD_DisplayStringLine(Line2,(unsigned char *)pucDisp);
		
		sprintf(pucDisp,"  Mode:%s",ppucMode[ucMode] );
		LCD_DisplayStringLine(Line4,(unsigned char *)pucDisp);
		
	}
	else
	{
		//LCD_Clear(Blue);
		LCD_DisplayStringLine(Line0,(unsigned char *)"       Para        ");
		sprintf(pucDisp,"  PA6:%d%%     ",pucDuty_Manu[0]);
		LCD_DisplayStringLine(Line2,(unsigned char *)pucDisp);
		sprintf(pucDisp,"  PA7:%d%%     ",pucDuty_Manu[1]);
		LCD_DisplayStringLine(Line4,(unsigned char *)pucDisp);
	}
	if(Capture)
	{
		sprintf(pucDisp,"  Freq:%4.1fHZ      ",1000000.0/Capture);
		LCD_DisplayStringLine(Line6,(unsigned char *)pucDisp);
		sprintf(pucDisp,"  Duty:%4.1f%%      ",100.0*Duty/Capture);
		LCD_DisplayStringLine(Line7,(unsigned char *)pucDisp);
	}
	else
	{
		sprintf(pucDisp,"  Freq:%4.1fHZ      ",0);
		LCD_DisplayStringLine(Line6,(unsigned char *)pucDisp);
		sprintf(pucDisp,"  Duty:%4.1f%%      ",0);
		LCD_DisplayStringLine(Line7,(unsigned char *)pucDisp);
	}
	sprintf(pucDisp,"  Capt1:%d      ",Capture);
	LCD_DisplayStringLine(Line8,(unsigned char *)pucDisp);
	sprintf(pucDisp,"  Duty1:%d      ",Duty);
	LCD_DisplayStringLine(Line9,(unsigned char *)pucDisp);
}
void LED_Proc(void)
{
	if(ucMode)
		ucLed |=1<<0;
	else
		ucLed &=~(1<<0);
	if(ucState )
		ucLed |=1<<1;
	else
		ucLed &=~(1<<1);
	LED_Disp(ucLed);
}
void ADC_Proc(void)
{
	static u32 uw100Ms_Temp=0;
	static u16 usR37_Old=0;
	if(uw100Ms_Temp != uw100Ms)
	{
		uw100Ms_Temp=uw100Ms;
		usR37=ADC2_Read();		
		if(ucMode )
		{
			if(usR37_Old != usR37)
			{
				usR37_Old=usR37;
				ucDuty_Auto=usR37*100/4095;
				//TIM PA6 PA7
				//htim3.Instance->CCR1=10000*ucDuty_Auto/100;
				//htim1.Instance->CCR1=10000*ucDuty_Auto;
				
			}
		}
	}
}
void KEY_Proc(void)
{
	uint8_t ucKey_Now;
	static uint8_t ucKey_Old=0;
	ucKey_Now=KEY_Read();
	//u8 pucDisp[25]={0};
	if(ucKey_Now!=ucKey_Old)
	{
		ucKey_Old=ucKey_Now;
		switch(ucKey_Now)
		{
			case 0:
				 
				break;
			case 1:
				ucState =!ucState ;
				LCD_Clear(Black);
				break;
			case 2:
				if(ucState)
				{
					pucDuty_Manu[0]+=10;
					if(pucDuty_Manu[0]>90) pucDuty_Manu[0]=10;
					//pucduty_Manu[]-->PA6,PA7
					//if(!ucMode)
					//	htim3.Instance->CCR1=10000*pucDuty_Manu[0]/100;
					MEM_Write(0, pucDuty_Manu,2);
				}
				break;
			case 3:
				if(ucState)
				{
					pucDuty_Manu[1]+=10;
					if(pucDuty_Manu[1]>90) pucDuty_Manu[1]=10;
					//pucduty_Manu[]-->PA6,PA7
					//if(!ucMode )
						//htim1.Instance->CCR1=5000*pucDuty_Manu[1]/100;
					//MEM_Write(1,pucDuty_Manu)
					MEM_Write(0, pucDuty_Manu,2);
				}
				break;
			case 4:
				ucMode =!ucMode;
				if(!ucMode)
				{
					//htim3.Instance->CCR1=10000*pucDuty_Manu[0]/100;
					//htim1.Instance->CCR1=5000*pucDuty_Manu[1]/100;
				}
				else
				{
					//htim3.Instance->CCR1=10000*ucDuty_Auto/100;
					//htim1.Instance->CCR1=5000*ucDuty_Auto;
				}
			break;
		}
	}
	
	
}
void TIM_Proc(void)
{
	uint32_t uwTim3Pulse,uwTim1Pulse;
	if(ucMode)
	{
		htim3.Instance->CCR1=ucDuty_Auto/100.0*10000;
		htim1.Instance->CCR1=ucDuty_Auto/100.0*5000;
	}
	else
	{
		htim3.Instance->CCR1=pucDuty_Manu[0]/100.0*10000;
		htim1.Instance->CCR1=pucDuty_Manu[1]/100.0*5000;						
	}
}
u8 IsNum(u8 ucChar)
{
	if(ucChar>=0x30 && ucChar<=0x39 )
		return 1;
	else
		return 0;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int rst=0;
	u8 ucPA6,ucPA7;
	//u8 *pucTemp[5]={0};
	u8 pucTemp[5]={0};
	u8 pucReply[4]={0};
	//pucIn  rst=sscanf(pucIn,"%c,%d,%s",&x,s)
	if(IsNum(pucIn[5]) && IsNum(pucIn[6]) && IsNum(pucIn[8]) && IsNum(pucIn[9]))
	{
		rst=sscanf(pucIn,"%4s:%d,%d",pucTemp,&ucPA6,&ucPA7);
		if(rst==3)
		{
			if(!strcmp(pucTemp,"AUTO") || !strcmp(pucTemp,"MANU"))
			{
				if(!strcmp(pucTemp,"AUTO"))
				{
					ucMode=1;
					strcpy(pucReply ,"OK1");
					//HAL_UART_Transmit_IT(&huart1,"OK1" ,3);
					HAL_UART_Transmit_IT(&huart1,pucReply ,3);
					//htim3.Instance->CCR1=ucDuty_Auto/100.0*10000;
					//htim1.Instance->CCR1=ucDuty_Auto/100.0*5000;
				}
				else
				{
					if(ucPA6 >=10 && ucPA6 <=90 && ucPA6 %10 ==0 && ucPA7 >=10 && ucPA7 <=90 && ucPA7 %10 ==0)
					{
						pucDuty_Manu[0]=ucPA6;
						pucDuty_Manu[1]=ucPA7;
						MEM_Write(0, pucDuty_Manu,2);
						ucMode=0;
						HAL_UART_Transmit_IT(&huart1,"OK2" ,3);
						//htim3.Instance->CCR1=pucDuty_Manu[0]/100.0*10000;
						//htim1.Instance->CCR1=pucDuty_Manu[1]/100.0*5000;			
					}
					else
					{
						strcpy(pucOut ,"Er3");
						HAL_UART_Transmit_IT(&huart1,pucOut ,3);
					}
				
				}
					
			}
			else
			{
				strcpy(pucOut ,"Er2");
				HAL_UART_Transmit_IT(&huart1,pucOut ,3);
			}
		}
		else
		{
			strcpy(pucOut ,"Er1");
			HAL_UART_Transmit_IT(&huart1,pucOut ,3);
	    //printf("Err");
		}
	}
	else
	{
		//HAL_UART_Transmit();  //printf()
		//HAL_UART_Transmit_IT(&huart1,"Err",3);
		strcpy(pucOut ,"Er0");
		HAL_UART_Transmit_IT(&huart1,pucOut ,3);
	}
	HAL_UART_Receive_IT(&huart1,pucIn,10);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance ==TIM2)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
		{
			Capture=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1)+1;
			Duty =htim2.Instance->CCR2+1;  
			//Duty=htim->Instance->CCR2 ; 
			//Duty=TIM2->CCR2    //CMSIS
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
