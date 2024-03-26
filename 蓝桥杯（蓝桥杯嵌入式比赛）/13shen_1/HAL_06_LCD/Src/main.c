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
vu32 uwTickms=0,uw100ms=0,uwOnes=0;
u8 ucLed=0;


u8 pucPsd[3]={'1','2','3'};
u8 pucPsd_Edit[3]={'@','@','@'};
u8 pucPsd_Temp[3];
u8 ucState=0;

vu32 Capture = 0; 
vu32  Duty=0;

u8  ucPsdErr3Times=0;
u32 uwLD2_StartTime;

u8 pucCmd[8]={0};

u16 pusADC1[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Lcd_Proc(void);
void Led_Proc(void);
u8 Key_Proc(void);
void Ztj_Proc(void);
void ADC1_Proc(void);
u8 IsNum(u8 InChar);
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
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start(&htim2,TIM_CHANNEL_2); 
	
	MEM_Read(0,pucPsd_Temp,3);
	if(IsNum(pucPsd_Temp[0]) && IsNum(pucPsd_Temp[1]) && IsNum(pucPsd_Temp[2]) )
	{
		pucPsd[0]=pucPsd_Temp[0];
		pucPsd[1]=pucPsd_Temp[1];
		pucPsd[2]=pucPsd_Temp[2];
	}
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_UART_Receive_IT(&huart1,pucCmd,7);
  while (1)
  {
    Ztj_Proc();
		Lcd_Proc();
		Led_Proc();	
		ADC1_Proc();
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
void Lcd_Proc(void)
{
	u8 pucDisp[25]={0};
	if(!ucState)
	{
		//LCD_Clear(Blue);
		LCD_DisplayStringLine(Line1,(unsigned char *)"        PSD        ");
		
		sprintf(pucDisp,"  B1:%c",pucPsd_Edit[0]);
		LCD_DisplayStringLine(Line3,(unsigned char *)pucDisp);
		
		sprintf(pucDisp,"  B2:%c",pucPsd_Edit[1]);
		LCD_DisplayStringLine(Line4,(unsigned char *)pucDisp);
		
		sprintf(pucDisp,"  B3:%c",pucPsd_Edit[2]);
		LCD_DisplayStringLine(Line5,(unsigned char *)pucDisp);
		
	}
	else
	{
		//LCD_Clear(Blue);
		LCD_DisplayStringLine(Line1,(unsigned char *)"       STA        ");
		
		sprintf(pucDisp,"  F:%4dHZ",2000);
		LCD_DisplayStringLine(Line3,(unsigned char *)pucDisp);
		
		sprintf(pucDisp,"  D:%2d%%",10);
		LCD_DisplayStringLine(Line4,(unsigned char *)pucDisp);
	
	}
	sprintf(pucDisp,"MCP:%4.1f%,R38:%4.1f%",pusADC1[0]*3.3/4095,pusADC1[1]*3.3/4095);
	LCD_DisplayStringLine(Line0,(unsigned char *)pucDisp);
	if(Capture)
	{
		sprintf(pucDisp,"  Freq:%4.2fHz",1000000.0/(Capture) );
		//sprintf(pucDisp,"  Freq:%dHz",1000000/(Capture) );
		LCD_DisplayStringLine(Line6,(unsigned char *)pucDisp);	
		sprintf(pucDisp,"  Duty:%2d%%",Duty*100/Capture );
		LCD_DisplayStringLine(Line7,(unsigned char *)pucDisp);
	}
	else
	{
		sprintf(pucDisp,"  Freq:%dHz",0);
		LCD_DisplayStringLine(Line6,(unsigned char *)pucDisp);	
		sprintf(pucDisp,"  Duty:%2d%%",0 );
		LCD_DisplayStringLine(Line7,(unsigned char *)pucDisp);
	}
	sprintf(pucDisp,"  Duty1:%d",Duty );
	LCD_DisplayStringLine(Line8,(unsigned char *)pucDisp);
	sprintf(pucDisp,"  Capt1:%d",Capture );
	LCD_DisplayStringLine(Line9,(unsigned char *)pucDisp);
}
void Led_Proc(void)
{
	u8 i=0;
	static u32 uwLedTime=0;
	if(ucPsdErr3Times==1 && (uwTickms-uwLD2_StartTime)<=5000)
	{
		if(uwLedTime!=uw100ms)
		{
			uwLedTime=uw100ms;
			ucLed ^=1<<1;			
		}
	}
	else
	{
		ucPsdErr3Times=0;	
		ucLed &=~(1<<1);
	}
	LED_Disp(ucLed);
}
void Ztj_Proc(void)
{
	u8 ucPsdOK=0;
	static u32 uwLD1_StartTime;
	if(!ucState)
	{
		ucPsdOK=Key_Proc();	
		if(ucPsdOK==1)
		{
			uwLD1_StartTime=uwTickms;
			ucState=1;
			LCD_Clear(Black);
			ucLed |=(1<<0);
			
			//Led_Out(ucLed);
			htim15.Instance->ARR=499;  ;
			htim15.Instance->CCR1=50;
		}
	}
	else
	{
		if(uwTickms-uwLD1_StartTime>=5000)
		{
			ucState =0;
			LCD_Clear(Black);
			ucLed &=~(1<<0);
			
			//Led_Out(ucLed);
			htim15.Instance->ARR=999;  ;
			htim15.Instance->CCR1=500;
		}
	}
}
void ADC1_Proc(void)
{
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,10)==HAL_OK)
		pusADC1[0]= HAL_ADC_GetValue(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1,10)==HAL_OK)
		pusADC1[1]= HAL_ADC_GetValue(&hadc1);
}
u8 Key_Proc(void)
{
	u8 ucKey_Val;
	static u8 ucKey_Old=0;
	static u32 ulKey_Time=0;
	static u16 uiErrNum=0;
	//static u8 ucOnce=0;
	ucKey_Val=KEY_Read();
	
	if(ucKey_Val !=ucKey_Old)
	{
		ucKey_Old=ucKey_Val;
		ulKey_Time=uwTickms;
		//ucOnce=0;
		switch(ucKey_Val)
		{
			case 1:
				if(pucPsd_Edit[0]=='@')
				{
					pucPsd_Edit[0]='0';
				}
				else
				{
					pucPsd_Edit[0]=0x30+((pucPsd_Edit[0]-0x30)+1) % 10;
				}
				break;		
			case 2:
				if(pucPsd_Edit[1]=='@')
				{
					pucPsd_Edit[1]='0';
				}
				else
				{
					//pucPsd_Edit[1]=0x30+((pucPsd_Edit[1]-0x30)+1) % 10;
					pucPsd_Edit[1]++;
					if(pucPsd_Edit[1]==0x3A) pucPsd_Edit[1]='0';
				}
				break;
			case 3:
				if(pucPsd_Edit[2]=='@')
				{
					pucPsd_Edit[2]='0';
				}
				else
				{
					pucPsd_Edit[2]=0x30+((pucPsd_Edit[2]-0x30)+1) % 10;
				}
				break;
			case 4:
				if(pucPsd[0]==pucPsd_Edit[0] && pucPsd[1]==pucPsd_Edit[1] && pucPsd[2]==pucPsd_Edit[2])
				{
					uiErrNum=0;
					pucPsd_Edit[0]='@';
					pucPsd_Edit[1]='@';
					pucPsd_Edit[2]='@';
					return 1;
				}
				else
				{
					uiErrNum++;
					pucPsd_Edit[0]='@';
					pucPsd_Edit[1]='@';
					pucPsd_Edit[2]='@';
					if(uiErrNum>=3)
					{
						ucPsdErr3Times=1;
						uwLD2_StartTime=uwTickms;						
					}
					
				}
				break;
		}
	}
	if(uwTickms-ulKey_Time>=800)
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
	return 0;
	
}
u8 IsNum(u8 InChar)
{
	if(InChar >=0x30 && InChar <=0x39)
		return 1;
	else
		return 0;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(IsNum(pucCmd[0]) && IsNum(pucCmd[1]) && IsNum(pucCmd[2]) && pucCmd[3]=='-' && IsNum(pucCmd[4]) && IsNum(pucCmd[5]) && IsNum(pucCmd[6]) )
	{
		if(pucCmd[0]==pucPsd[0] && pucCmd[1]==pucPsd[1] && pucCmd[2]==pucPsd[2])
		{
			pucPsd[0]=pucCmd[4];
			pucPsd[1]=pucCmd[5];
			pucPsd[2]=pucCmd[6];
			HAL_UART_Transmit_IT(&huart1,"OK",2);	
			MEM_Write(0,pucPsd,3);
			MCP_Write((pucPsd[2]-'0')*10);   //3 bei
		}
		else
			HAL_UART_Transmit_IT(&huart1,"err2",4);	;	//debug
	}
	else
	{
			HAL_UART_Transmit_IT(&huart1,"err1",4);		//debug
	}
	HAL_UART_Receive_IT(&huart1,pucCmd,7);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{	
	if(htim->Instance ==TIM2)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
		{
			Capture=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1)+1;			
			Duty=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2)+1;
			
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
