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
#include "i2c.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
vu32 ulTickms=0,ulHalfs=0,ulOnes=0;
u8 ucLed=0;

u8 ucState=0;				//0,123£¬4£¬5
u8 pucTime[15]={0};
u8 ucID=0;   //display need +1
vu32 ulTime;
u8 ucHour,ucMin,ucSec;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LCD_Proc(void);
void LED_Proc(void);
void KEY_Proc(void);
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
	u8 m;
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Blue);
	LCD_SetBackColor(Blue);
	LCD_SetTextColor(White);
	
	MEM_Read(0,pucTime,15);
	for(m=0;m<15;m++)
	{
		if(m % 3==0)
			if(pucTime[m]>=24) pucTime[m]=0;
		else 
			if(pucTime[m]>=60) pucTime[m]=0;
	}
	ucID=0;
	ucState=0;
	ulTime =pucTime[ucID*3+0]*3600+pucTime[ucID*3+1]*60+pucTime[ucID*3+2];
	
  while (1)
  {
    LCD_Proc();
		LED_Proc();
		//Buzz_proc();
		KEY_Proc();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
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
void LCD_Proc(void)
{
	u8 pucDisp[25]={0};
	//u8 *str2="hello";
	u8 ucDispH,ucDispM,ucDispS;
		//LCD_Clear(Blue);
		//LCD_DisplayChar(Line0,319-16*(2-1),*str2);
		ucDispH=ulTime / 3600;
		ucDispM=(ulTime % 3600)/60;
		ucDispS=ulTime %60;
		sprintf(pucDisp,"     No.%d",ucID+1);
		LCD_DisplayStringLine(Line2,(unsigned char *)pucDisp);
		
		if(ucState==0 || ucState>3)
		{
			sprintf(pucDisp,"     %02d:%02d:%02d",ucDispH,ucDispM,ucDispS);
			LCD_DisplayStringLine(Line3,(unsigned char *)pucDisp);
		}
		if(ucState ==1)
		{	
			LCD_SetTextColor(Red);
			LCD_DisplayChar(Line3,319-16*(5+3*0),0x30+ucDispH/10);
			LCD_DisplayChar(Line3,319-16*(5+3*0+1),0x30+ucDispH %10);			
			LCD_SetTextColor(White);
			LCD_DisplayChar(Line3,319-16*(5+3*0+2),':');
			LCD_DisplayChar(Line3,319-16*(5+3*1),0x30+ucDispM/10);
			LCD_DisplayChar(Line3,319-16*(5+3*1+1),0x30+ucDispM%10);
			LCD_DisplayChar(Line3,319-16*(5+3*1+2),':');
			LCD_DisplayChar(Line3,319-16*(5+3*2),0x30+ucDispS/10);
			LCD_DisplayChar(Line3,319-16*(5+3*2+1),0x30+ucDispS%10);
		}
		else if(ucState==2)
		{
			LCD_DisplayChar(Line3,319-16*(5+3*0),0x30+ucDispH/10);
			LCD_DisplayChar(Line3,319-16*(5+3*0+1),0x30+ucDispH %10);		
			LCD_DisplayChar(Line3,319-16*(5+3*0+2),':');
			LCD_SetTextColor(Red);
			LCD_DisplayChar(Line3,319-16*(5+3*1),0x30+ucDispM/10);
			LCD_DisplayChar(Line3,319-16*(5+3*1+1),0x30+ucDispM%10);
			LCD_SetTextColor(White);
			LCD_DisplayChar(Line3,319-16*(5+3*1+2),':');
			LCD_DisplayChar(Line3,319-16*(5+3*2),0x30+ucDispS/10);
			LCD_DisplayChar(Line3,319-16*(5+3*2+1),0x30+ucDispS%10);
		}			
		else if(ucState==3)
		{
			LCD_DisplayChar(Line3,319-16*(5+3*0),0x30+ucDispH/10);
			LCD_DisplayChar(Line3,319-16*(5+3*0+1),0x30+ucDispH %10);		
			LCD_DisplayChar(Line3,319-16*(5+3*0+2),':');
			LCD_DisplayChar(Line3,319-16*(5+3*1),0x30+ucDispM/10);
			LCD_DisplayChar(Line3,319-16*(5+3*1+1),0x30+ucDispM%10);
			LCD_DisplayChar(Line3,319-16*(5+3*1+2),':');
			LCD_SetTextColor(Red);
			LCD_DisplayChar(Line3,319-16*(5+3*2),0x30+ucDispS/10);
			LCD_DisplayChar(Line3,319-16*(5+3*2+1),0x30+ucDispS%10);
			LCD_SetTextColor(White);
		}
			
		
		switch (ucState )
		{
			case 0:
				strcpy(pucDisp,"     Standby");
				//sprintf(pucDisp,"     %s","Standby");
				break;
			case 1:
			case 2:
			case 3:
				strcpy(pucDisp,"     Setting");
				break;
			case 4:
				strcpy(pucDisp,"     Running");
				break;
			case 5:
				strcpy(pucDisp,"     Pause  ");
				break;
		}	
		LCD_DisplayStringLine(Line4,(unsigned char *)pucDisp);
}
void LED_Proc(void)
{
	u8 i=0;
	static u32 ulLedTime=0;
	if(ucState==4)
	{
		if(ulLedTime!=ulOnes)
		{
			ulLedTime=ulOnes;
			ucLed ^=1<<0;			
		}
	}
	else
		ucLed &=~(1<<0);
	LED_Disp(ucLed);
}
void TIM_Proc0(void)
{
	if(ucState==4)
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	else
		HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
}
void KEY_Proc(void)
{
	u8 ucKey_Val;
	static u8 ucKey_Old=0;
	static u32 ulKey_Time=0;
	static u8 ucOnce=0;
	ucKey_Val=KEY_Read();
	if(ucKey_Val !=ucKey_Old)
	{
		ucKey_Old=ucKey_Val;
		ulKey_Time=ulTickms;
		ucOnce=0;
		switch(ucKey_Val)
		{
			case 1:				
					LCD_Clear(Blue);
					ucID=(ucID+1) %5;
					ucState=0;
					ulTime =pucTime[ucID*3+0]*3600+pucTime[ucID*3+1]*60+pucTime[ucID*3+2];				
					
				break;
			case 2:
				if(!ucState)
				{
					ucState=1;
					ucHour=ulTime / 3600;
					ucMin=(ulTime % 3600)/60;
					ucSec=ulTime %60;
				}
				else if(ucState <4)
				{
					ucState=(ucState-1+1) % 3+1;
				}
				break;
			case 3:
				switch(ucState)
				{
					case 1:
						ucHour=(ucHour+1)% 24;
						ulTime=ucHour*3600+ucMin*60+ucSec;
						break;
					case 2:
						ucMin=(ucMin+1)% 60;
						ulTime=ucHour*3600+ucMin*60+ucSec;
						break;
					case 3:
						ucSec=(ucSec+1)% 60;
						ulTime=ucHour*3600+ucMin*60+ucSec;
						break;
				}
				break;
			case 4:
				if(ucState!=4)
				{
					ucState =4;
					
				}
				else
				{
					ucState =5;
				
				}
				break;
		}
	}
	if(ulTickms-ulKey_Time>=800)
	{
		switch(ucKey_Old)
		{
			case 1:
				
				break;
			case 2:
				if(ucState>0 && ucState <4 )
				{						
					pucTime[ucID*3+0]=ulTime / 3600;
					pucTime[ucID*3+1]=(ulTime % 3600)/60;
					pucTime[ucID*3+2]=ulTime %60;
					MEM_Write(0,pucTime,15);					
					
					ucState =0;
				}
				
				break;
			case 3:
					switch(ucState)
					{
						case 1:
							ucHour=(ucHour+1)% 24;
							ulTime=ucHour*3600+ucMin*60+ucSec;
							break;
						case 2:
							ucMin=(ucMin+1)% 60;
							ulTime=ucHour*3600+ucMin*60+ucSec;
							break;
						case 3:
							ucSec=(ucSec+1)% 60;
							ulTime=ucHour*3600+ucMin*60+ucSec;
							break;
					}
					break;
			case 4:
				if(ucState ==4) 
				{
					ucState =0;
					
				}
				break;
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
