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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ble_remote.h"
#include <stdio.h>
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
uint8_t data[11];
uint8_t chassis_xyz[8];
int16_t chassis[4];
int16_t xyz[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
/*底盘麦轮速度初始化*/
uint8_t kp=20;
int PWMA_1=0,PWMA_2=0;  
int PWMB_1=0,PWMB_2=0;
int PWMC_1=0,PWMC_2=0;
int PWMD_1=0,PWMD_2=0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*舵机角度初始化*/
int angle_pwm1=1000;  /*45°*/
int angle_pwm2=1500;
int angle_pwm3=1800;/*
p*/
int angle_pwm4=2500; /*180°*/


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
const remote_t *r=get_remote_control_point();
for(int i=0;i<4;i++)
	{ 
		chassis[i]=0;
	
	}
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM15_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_TIM17_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_DMA(&huart4,data,11); 

/*开底盘PWM*/
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
  HAL_TIM_Base_Start(&htim12);
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2); 
  HAL_TIM_Base_Start(&htim15);
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim17);
  HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);

/*开舵机PWM*/
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	
	  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
	  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	
	  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
  {		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		/********* 将每个数字处理求和 *********/
    
	  xyz[0]=r->rocker[0].y_position;/*X*/
	  xyz[1]=r->rocker[0].x_position;/*Y*/
    xyz[2]=r->rocker[1].x_position;/*Z*/
   /*********四个电机的速度计算 *********/ 
    chassis[0]=xyz[0]+xyz[1]+xyz[2];
    chassis[1]=xyz[0]-xyz[1]+xyz[2];
    chassis[2]=-xyz[0]+xyz[1]+xyz[2];
    chassis[3]=-xyz[0]-xyz[1]+xyz[2];
		 /*********PWM输出赋值*********/
		if(r->Switch[3]==0) kp=10;
		if(r->Switch[3]==1) kp=50;
   
    if(chassis[0]>0)    PWMA_1=kp*chassis[0], PWMA_2=0;
    else                PWMA_1=0, PWMA_2=-kp*chassis[0];
    if(chassis[1]>0)    PWMB_1=kp*chassis[1], PWMB_2=0;
    else                PWMB_1=0, PWMB_2=-kp*chassis[1];  
    if(chassis[2]>0)    PWMC_1=kp*chassis[2], PWMC_2=0;
    else                PWMC_1=0, PWMC_2=-kp*chassis[2];
    if(chassis[3]>0)    PWMD_1=kp*chassis[3], PWMD_2=0;
    else                PWMD_1=0, PWMD_2=-kp*chassis[3]; 
     /********* PWM输出 *********/
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,PWMA_1);/* M1+ */
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,PWMA_2);/* M1- */
    __HAL_TIM_SetCompare(&htim15,TIM_CHANNEL_2,PWMB_1);/* M2+ */
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,PWMB_2);/* M2- */
    __HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,PWMC_2);/* M3+ */
    __HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_2,PWMC_1);/* M3- */ 
    __HAL_TIM_SetCompare(&htim15,TIM_CHANNEL_1,PWMD_2);/* M4+ */
    __HAL_TIM_SetCompare(&htim17,TIM_CHANNEL_1,PWMD_1);/* M4- */
		
		
		/*舵机1：由Button1、2按键控制角度加减*/
		if(r->Button[0]==1&&angle_pwm1>=500)
		{   
		 angle_pwm1=angle_pwm1-25;		
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,angle_pwm1);
		}
		
		else if(r->Button[1]==1&&angle_pwm1<=2500)
		{  
			angle_pwm1=angle_pwm1+25;	
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,angle_pwm1);
		
		}
		
		HAL_Delay(10);
		/*舵机2：由Button3、4按键控制角度加减*/
			if(r->Button[2]==1&&angle_pwm2>=500)
		{   
		 angle_pwm2=angle_pwm2-50;		
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,angle_pwm2);
		}
		
		else if(r->Button[3]==1&&angle_pwm2<=1800)
		{  
			angle_pwm2=angle_pwm2+50;	
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,angle_pwm2);
		
		}
		
		HAL_Delay(10);
		
		/*舵机3：由Switch1、2按键控制角度加减*/
			if(r->Switch[0]==1&&angle_pwm3>=500)
		{   
		 angle_pwm3=angle_pwm3-40;		
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,angle_pwm3);
		}
		
		else if(r->Switch[1]==1&&angle_pwm3<=2500)
		{  
			angle_pwm3=angle_pwm3+40;	
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,angle_pwm3);
		
		}
		HAL_Delay(10);
		
		/*舵机4：由Switch3、4按键控制角度加减*/
			if(r->Switch[2]==1&&angle_pwm4>=1500)
		{   
		 angle_pwm4=angle_pwm4-50;		
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,angle_pwm4);
		}
		
		else if(r->Switch[2]==0&&angle_pwm4<=2500)
		{  
			angle_pwm4=angle_pwm4+50;	
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,angle_pwm4);
		
		}
		
		
		
	  HAL_Delay(30);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART4)
	{
		uart_to_remote(data);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
