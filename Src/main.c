#define US1_Trigger_Port GPIOC
#define US1_Trigger_Pin  GPIO_PIN_8
// PA9 US1 Echo
#define US1_Echo_Port GPIOC
#define US1_Echo_Pin  GPIO_PIN_6


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
	
  */
	uint32_t us1_duration=1;
uint8_t state;
uint32_t us1_distance=1;
int tiempo=0;
int estado=0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){  //INTERRUPCIÓN EN EL PIN A1

	if(GPIO_Pin == GPIO_PIN_1){   //SI DETECTA QUE LA INTERRUPCIÓN CORRESPONDE AL PIN1, ALTERNA EL ESTADO DEL LED
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_9);
		}
}


	
int main(void)
{

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
 
  while (1)
  {
		

		HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);   //CONFIGURACIÓN DE LA INTERRUPCIÓN
		HAL_NVIC_EnableIRQ(EXTI1_IRQn);
				 
		HAL_Delay(50);         //PARA QUE TOME MEDIDAS CADA 50MS LA DISTANCIA A LA QUE SE ENCUENTRA EL OBJETO
		
		//ENVÍA PULSOS AL TRIGGER, QUE ACTIVA EL FUNCIONAMIENTO DEL SENSOR
		//COMO EL PULSO NECESARIO PARA QUE ESTE SENSOR FUNCIONE ES DE 10MILISEGUNDOS, LE PONEMOS UN DELAY
		//SI EL PULSO FUERA MENOR A 10MILISEGUNDOS EL SENSOR NO FUNCIONARÍA (TUVIMOS PROBLEMAS PARA AVERIGUAR ESTA CARACTERÍSTICA DEL SENSOR)
	  HAL_GPIO_WritePin(US1_Trigger_Port, US1_Trigger_Pin, GPIO_PIN_SET);   
    HAL_Delay(10);
    HAL_GPIO_WritePin(US1_Trigger_Port, US1_Trigger_Pin, GPIO_PIN_RESET);
		
		
   //MIENTRAS NO DETECTE UNA SEÑAL DE SALIDA (UN OBJETO), SIGUE LEYENDO
	 //EL PIN DE SALIDA EL ULTRASONIDO
   do {
			state = HAL_GPIO_ReadPin(US1_Echo_Port, US1_Echo_Pin); 
    } while (state == RESET);																
	
		
    uint32_t us1_start = HAL_GetTick();		//UNA VEZ DETECTA UNA SEÑAL(ECHO) EN ALTO, MIDE LA LONGITUD DEL PULSO
		
    
    do{																										//SIGUE LEYENDO LA SEÑAL (ECHO) HASTA QUE LA SEÑAL SE PONE EN NIVEL BAJO
      state = HAL_GPIO_ReadPin(US1_Echo_Port, US1_Echo_Pin);
  } while (state == SET);																	
	
		
    uint32_t us1_end = HAL_GetTick();											//CUENTA EL TIEMPO QUE TARDA EN PONERSE EN BAJO LA SEÑAL
		
	
			// EXPLICACIÓN GRÁFICA DEL FUNCIONAMIENTO DEL SENSOR HC-SR04
		//         -----------
		//        |           |
		//        |           |
		//--------            --------------
	  //
	  //<------->             AQUI CUENTA CUANDO EL PULSO SE PONE EN ALTO POR PRIMERA VEZ, ES LA SEÑAL US1_START
		//<-------------------> AQUI EMPIEZA A CONTAR CUANDO PULSO BAJA A NIVEL BAJO, US1_END
	
		//UNA VEZ TENEMOS LA LONGITUD DE AMBOS PULSOS, LA RESTAMOS PARA OBTENER EL TIEMPO QUE HA ESTADO LA SEÑAL
		//ECHO EN ALTO (ES DECIR, LO QUE HA TARDADO EN REBOTAR LA SEÑAL)
    us1_duration = us1_end - us1_start;

	
		
		//SI LA SEÑAL ES MENOR 2 MICROSEGUNDOS (ESTA A CORTA DISTANCIA)
		if(us1_duration<2){
			tiempo=0;   //PONEMOS LA VARIABLE QUE GUARDA EL VALOR DEL TEMPORIZADOR A 0
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);  //ENCENDEMOS EL LED
			HAL_TIM_Base_Start_IT(&htim2);      //EMPIEZA A CONTAR EL TEMPORIZADOR
		}else{   //SI LA SEÑAL ES MAYOR DE 2 MICROSEGUNDOS (ESTÁ A UNA LARGA DISTANCIA)
			tiempo = __HAL_TIM_GET_COUNTER(&htim2);   //GUARDAMOS EL VALOR DEL TEMPORIZADOR EN LA VARIABLE TIEMPO
			
			if(tiempo>3000){			//SI EL TEMPORIZADOR A ALCANZADO 3 SEGUNDOS SE APAGA EL LED
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);  
			}
		}
    /* USER CODE BEGIN 3 */
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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
