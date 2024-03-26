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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "time.h"
#include "string.h"
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
vu8 ucLed=0;
u8 ucState=0;
u8 ucCNBR,ucVNBR,ucIDLE;

float MyRate[2]={3.5,2.0};
u8 ucA7PWM=0;

struct MyCar
{
	u8 ucCarType;
	u8 pucCarNo[5];
	struct tm tmIn;
};
struct MyCar CarKu[8]={{0,"",},{0,"",},{0,"",},{0,"",},{0,"",},{0,"",},{0,"",},{0,"",}};

u8 pucCmd[23]={0};
u8 pucRpy[23]={0};


vu32 ulTickms=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void LCD_Proc(void);
void LED_Proc(void);
void KEY_Proc(void);

u8 LeapYear(u16 InYear);
u8 Mondays(u16 InYear,u8 InMonth);
u8 TimeOK(u8 Inyear,u8 InMonth,u8 InDay,u8 InHour,u8 InMin,u8 InSec);
s16 InCar(u8 ucCarType_L,u8 *pucCarNo_L,struct tm tmInOut);
void CKStatus_Proc();
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	HAL_UART_Receive_IT(&huart1,pucCmd,22);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		KEY_Proc();
		CKStatus_Proc();
		LCD_Proc();
		LED_Proc();
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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 35;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void LCD_Proc(void)
{
	u8 pucDisp[25]={0};
	
	if(!ucState)
	{		
		//LCD_Clear(Blue);
		LCD_DisplayStringLine(Line1,(unsigned char *)"        Data        ");
		
		sprintf(pucDisp,"  CNBR:%d",ucCNBR);
		LCD_DisplayStringLine(Line3,(unsigned char *)pucDisp);
		
		sprintf(pucDisp,"  VNBR:%d",ucVNBR);
		LCD_DisplayStringLine(Line5,(unsigned char *)pucDisp);
		
		sprintf(pucDisp,"  IDLE:%d",ucIDLE);
		LCD_DisplayStringLine(Line7,(unsigned char *)pucDisp);
		
	}
	else
	{
		//LCD_Clear(Blue);
		LCD_DisplayStringLine(Line1,(unsigned char *)"       Para        ");
		
		sprintf(pucDisp,"  CNBR:%-3.2f",MyRate[0]);
		LCD_DisplayStringLine(Line3,(unsigned char *)pucDisp);		
		
		sprintf(pucDisp,"  VNBR:%-3.2f",MyRate[1]);
		LCD_DisplayStringLine(Line5,(unsigned char *)pucDisp);
	}
	
}
void LED_Proc(void)
{
	if(!ucA7PWM)	
		ucLed &=~(1<<1);
	else
		ucLed |=(1<<1);
	if(ucIDLE>0)
		ucLed |=(1<<0);
	else
		ucLed &=~(1<<0);
	LED_Disp(ucLed);
}

void KEY_Proc(void)
{
	u8 ucKey_Val;
	static u8 ucKey_Old=0;
	static u32 ulKey_Time=0;
	ucKey_Val=KEY_Read();
	if(ucKey_Val !=ucKey_Old)
	{
		ucKey_Old=ucKey_Val;
		ulKey_Time=ulTickms;
		switch(ucKey_Val)
		{
			case 1:
				if(!ucState)
				{
					ucState=1;
					LCD_Clear(Black);
				}
				else
				{
					ucState=0;
					LCD_Clear(Black);
				}
				break;
			case 2:
				if(ucState==1)
				{
					MyRate[0] +=0.5;
					MyRate[1] +=0.5;
				}
				break;
			case 3:
				if(ucState==1)
				{
					MyRate[0] -=0.5;
					if(MyRate[0] <=0)
						MyRate[0] =0.5;
					MyRate[1] -=0.5;
					if(MyRate[1] <=0)
						MyRate[1] =0.5;
				}
				break;
			case 4:
				if(!ucA7PWM)
				{
					ucA7PWM=1;
					HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); 
				}
				else
				{
					ucA7PWM=0;
					HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2); 
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
				
				break;
			case 3:
					
				break;
			case 4:
				
				break;
		}
	}
	
}

u8 LeapYear(u16 InYear)
{
	if (InYear % 400==0 || InYear %4==0 && InYear % 100 !=0)
		return 1;
	else
		return 0;
}
u8 Mondays(u16 InYear,u8 InMonth)
{
	u8 days[12]={31,28,31,30,31,30,31,31,30,31,30,31};
	if (LeapYear(InYear)) 
		days[1]=29;
	return days[InMonth-1];
}
u8 TimeOK(u8 Inyear,u8 InMonth,u8 InDay,u8 InHour,u8 InMin,u8 InSec)
{
	//Inyear should >=1900
	if (InMonth <1 || InMonth >12)
		return 0;
	else if(InDay<1 || InDay > Mondays(2000+Inyear,InMonth))
		return 0;
	else if(InHour>23)
		return 0;
	else if(InHour>59)
		return 0;
	else if(InSec>59)
		return 0;
	else
		return 1;
}
//hour,0,-1
//struct tm tmInOut1,tmInOut2;
//u8 *pucCarNo_L1,*pucCarNo_L2;
//strcpy(pucCarNo_L1,pucCarNo_L2);
//tmInOut1=tmInOut2;
s16 InCar(u8 ucCarType_L,u8 *pucCarNo_L,struct tm tmInOut)
{
	u8 i,ucFlag,idleI,idleFlag;	
	time_t tSecs;
	u16 uiHours;
	idleFlag=0;	//wheather find first empty car position or not?
	for(i=0;i<8;i++)
	{
		if(!idleFlag && CarKu[i].ucCarType==0)
		{
			idleFlag=1;
			idleI=i;	//the first empty car position
		}
		if(CarKu[i].ucCarType>0 &&!strcmp(pucCarNo_L,CarKu[i].pucCarNo))
		{
			tSecs=mktime(&tmInOut)-mktime(&CarKu[i].tmIn);
			uiHours=tSecs /3600+1;
			CarKu[i].ucCarType=0;
			//CarKu[i].pucCarNo="";
			strcpy(CarKu[i].pucCarNo,"");
			return uiHours;				
		}
	}
	
	if(idleFlag==1)
	{
		CarKu[idleI].ucCarType=ucCarType_L;
		strcpy(CarKu[idleI].pucCarNo,pucCarNo_L); 
		CarKu[idleI].tmIn=tmInOut;
		return 0;
	}
	else
		return -1;
}

void CKStatus_Proc()
{
	u8 i,CNums,VNums,INums;
	CNums=0;
	VNums=0;
	INums=0;
	for(i=0;i<8;i++)
	{
		switch (CarKu[i].ucCarType)
		{
			case 0:
				INums++;
				break;
			case 1:
				CNums++;
				break;
			case 2:
				VNums++;
				break;
		}
	}
	ucCNBR=CNums;
	ucVNBR=VNums;
	ucIDLE=INums;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	u8 pucType[5],pucCar[5],year,month,day,hour,min,sec;
	u8 ucCarType;
	struct tm tmTime;
	s16 siStopHours;
	float fMoney;
	
	int result;	
    	
			result=sscanf(pucCmd,"%4s:%4s:%2d%2d%2d%2d%2d%2d",pucType,pucCar,&year,&month,&day,&hour,&min,&sec);
			if(result==8)
			{				
				
				if((!strcmp(pucType,"CNBR") || !strcmp(pucType,"VNBR")) && TimeOK(year,month,day ,hour,min,sec)==1)
				{
					if (!strcmp(pucType,"CNBR"))
						ucCarType=1;
					else
						ucCarType=2;
					tmTime.tm_year=2000+year-1900;
					tmTime.tm_mon=month-1;
					tmTime.tm_mday=day;
					tmTime.tm_hour=hour;
					tmTime.tm_min =min;
					tmTime.tm_sec=sec;
					siStopHours=InCar(ucCarType,pucCar,tmTime);
					if (siStopHours>0)
					{
						fMoney=MyRate[ucCarType-1]*siStopHours;
						
						sprintf(pucRpy,"%s:%s:%d:%-4.2f",pucType,pucCar,siStopHours,fMoney);
						HAL_UART_Transmit_IT(&huart1,pucRpy,strlen(pucRpy));
					}
					else if(siStopHours<0)
						HAL_UART_Transmit_IT(&huart1,"Error2",6);
					else
						HAL_UART_Transmit_IT(&huart1,"In",2);;   //debug remove
				}					
				else
					HAL_UART_Transmit_IT(&huart1,"Error1",6);
			}
			else
			{
				HAL_UART_Transmit_IT(&huart1,"Error",5);
			}		   
    
	HAL_UART_Receive_IT(&huart1,pucCmd,22);
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
