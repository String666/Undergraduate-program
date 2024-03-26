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
vu32 uwTickMs=0,uwTick100Ms=0;

u8 ucState=0; //0--shop,1--price,2--rep
u8 ucEnter=0;  //0--normal 5%  1--30%
u32 uwEnterMs=0;

u8 pucShop[2]={0,0};
u8 pucPrice[2]={10,10};			//1,0,1,0    x10
u8 pucRep[2]={10,10};

u8 ucUARTCmd;
u8 pucUARTFee[25]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Key_Proc(void);
void LCD_Proc(void);
void LED_PWM_Proc(void);

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
	u8 pucI2C_Read[5];
	u8 pucI2C_Write[5];
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	MEM_Read(0,pucI2C_Read,5);
	if(pucI2C_Read[4]!=1)
	{
		pucRep[0]=10;
		pucRep[1]=10;
		pucPrice[0]=10;
		pucPrice[1]=10;
		pucI2C_Write[0]=10;
		pucI2C_Write[1]=10;
		pucI2C_Write[2]=10;
		pucI2C_Write[3]=10;
		pucI2C_Write[4]=1;
		MEM_Write(0,pucI2C_Write,5);
	}
	else
	{
		if(pucI2C_Read[2]>=10 && pucI2C_Read[2]<=20 && pucI2C_Read[3]>=10 && pucI2C_Read[3]<=20)
		{
			pucRep[0]=pucI2C_Read[0];
			pucRep[1]=pucI2C_Read[1];
			pucPrice[0]=pucI2C_Read[2];
			pucPrice[1]=pucI2C_Read[3];
		}
		else
		{
			pucRep[0]=10;
			pucRep[1]=10;
			pucPrice[0]=10;
			pucPrice[1]=10;
			pucI2C_Write[0]=10;
			pucI2C_Write[1]=10;
			pucI2C_Write[2]=10;
			pucI2C_Write[3]=10;
			pucI2C_Write[4]=1;
			MEM_Write(0,pucI2C_Write,5);
		}
	}
	HAL_UART_Receive_IT(&huart1,&ucUARTCmd,1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Key_Proc();
		LCD_Proc();
		LED_PWM_Proc();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Key_Proc(void)
{
	//bt1-->toggle LD1
	//bt2-->LD2,3,4 ---ON
	//bt3-->LD2,3,4---OFF
	uint8_t ucKey_Now;
	static uint8_t ucKey_Old=0;
	ucKey_Now=KEY_Read();
	u8 pucDisp[25]={0};
	float fFee;
	if(ucKey_Now!=ucKey_Old)
	{
		ucKey_Old=ucKey_Now;
	}
	else
	{
		ucKey_Now=0;
	}
	switch(ucKey_Now)
	{
		case 1:			
			ucState =(ucState+1) % 3;  
			LCD_Clear(Black);
			//ucState =! ucState;			
		  /*if(!ucState )
				ucState =1;
			else
				ucState =0;*/
			break;
		case 2:
			switch(ucState)
			{
				case 0:
					pucShop[0]++;
					if(pucShop[0]>pucRep[0])
						pucShop[0]=0;
					break;
				case 1:
					pucPrice[0]++;
					if(pucPrice[0]>20)
						pucPrice[0]=10;
					MEM_Write (2,pucPrice,1);
					break;
				case 2:
					pucRep[0]++;
					MEM_Write (0,pucRep,1);
					break;
			}
			break;
		case 3:
			switch(ucState)
			{
				case 0:
					pucShop[1]++;
					if(pucShop[1]>pucRep[1])
						pucShop[1]=0;
					break;
				case 1:
					pucPrice[1]++;
					if(pucPrice[1]>20)
						pucPrice[1]=10;
					MEM_Write (3,&(pucPrice[1]),1);
					break;
				case 2:
					pucRep[1]++;
					MEM_Write (1,&(pucRep[1]),1);
					break;
			}
			break;
		case 4:
			fFee=pucShop[0]*pucPrice[0]/10.0+pucShop[1]*pucPrice[1]/10.0;
			pucRep[0] -=pucShop[0];
			pucRep[1] -=pucShop[1];
			
			sprintf(pucUARTFee,"X:%d,Y:%d,Z:%3.1f",pucShop[0],pucShop[1],fFee);			
			HAL_UART_Transmit_IT(&huart1,(uint8_t *)pucUARTFee,strlen(pucUARTFee));
			if(pucShop[0] !=0 || pucShop[1] !=0)
			{
				MEM_Write (0,pucRep ,2);
			}
			pucShop[0]=0;
			pucShop[1]=0;			
			ucEnter=1;
			uwEnterMs=uwTickMs;
			htim2.Instance->CCR2=150;
		break;
	}
	//sprintf(pucDisp,"   Press Btn%d",ucKey_Now);
	//LCD_DisplayStringLine(Line3,(u8 *)pucDisp);
}
void LCD_Proc(void)
{
	u8 pucDisp[25]={0};
	if(!ucState)
	{		
		LCD_DisplayStringLine(Line1,(u8 *)"        SHOP        ");
		sprintf(pucDisp,"     X:%d    ",pucShop[0]);
		LCD_DisplayStringLine(Line2,(u8 *)pucDisp);
		sprintf(pucDisp,"     Y:%d        ",pucShop[1]);
		LCD_DisplayStringLine(Line3,(u8 *)pucDisp);
	}
	else if(ucState==1)
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        PRICE        ");
		sprintf(pucDisp,"     X:%3.1f",pucPrice[0]/10.0);
		LCD_DisplayStringLine(Line2,(u8 *)pucDisp);
		sprintf(pucDisp,"     Y:%3.1f",pucPrice[1]/10.0);
		LCD_DisplayStringLine(Line3,(u8 *)pucDisp);
	}
	else
	{
		LCD_DisplayStringLine(Line1,(u8 *)"        REP        ");
		sprintf(pucDisp,"     X:%d    ",pucRep[0]);
		LCD_DisplayStringLine(Line2,(u8 *)pucDisp);
		sprintf(pucDisp,"     Y:%d     ",pucRep[1]);
		LCD_DisplayStringLine(Line3,(u8 *)pucDisp);
	}	
}

void LED_PWM_Proc(void)
{
	static u32 uwTick100Ms_Temp=0;
	if(ucEnter)
	{
		ucLed |=1<<0;
		if(uwTickMs-uwEnterMs>=5000)
		{
			ucEnter=0;
		}
	}
	else
		ucLed &=1<<0;
	if(pucRep[0]==0 &&pucRep[1]==0)
	{
		if(uwTick100Ms_Temp !=uwTick100Ms)
		{
			uwTick100Ms_Temp =uwTick100Ms;
			ucLed ^=1<<1;
		}
	}
	LED_Disp(ucLed);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(ucUARTCmd=='?')
	{
		sprintf(pucUARTFee,"X:%3.1f,Y:%3.1f",pucPrice[0]/10.0,pucPrice[1]/10.0);			
		HAL_UART_Transmit_IT(&huart1,(uint8_t *)pucUARTFee,strlen(pucUARTFee));
	}
	else
		HAL_UART_Transmit_IT(&huart1,"Err",3);
	HAL_UART_Receive_IT(&huart1,&ucUARTCmd,1);
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
