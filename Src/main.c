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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"
#include "mfccFunc.h"
#include "constants.h"
#include "classification.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const int bufferSize = 4000;//2000
const int speechSize = 4000;
const int energyFrame = 40;
float mfcc[24*13];
float results[10];  
static float speech[speechSize]; //0.5s of speech
static uint32_t ADCBuffer[bufferSize];

int callibration = 0;
float temp_thresh = 0;
float set_thresh = 0;
int DMA_Active = 0;
int STOP_DMA = 0;
float energy = 0;
int start = -1;
int speechCap = 0;
int fillCounter = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int energyDetect(int index);
void mahalaTrans(float *Speech, int size);
void UART_Transmit_F(float *array, int size);
void UART_Transmit_R(float *array, int size);
void UART_Transmit_U(uint32_t *array, int size);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* */
int energyDetect(int index){	
		if(callibration < 4){ //While callibrating, we just add up all the energy of every frame and then get the average to set the threshold
			
					temp_thresh = 0;
					for(int i = index; i < index+(bufferSize/2)-energyFrame;i+=20){ //take windows of 40 & increase by 20 (half buffer is 2000)
							energy = 0;
							for(int j = 0; j < energyFrame; j++){
									energy = energy + ADCBuffer[i+j]*ADCBuffer[i+j];
							}
							temp_thresh = temp_thresh + energy;
					}
					set_thresh = set_thresh + temp_thresh;
			
			
		}else{
			for(int i = index; i < index+(bufferSize/2)-energyFrame;i+=20){ //take windows of 40 & increase by 20 (half buffer is 2000)
				
				if(start == -1){ //if speech hasn't been detected when energyDetect is called we look for when we can set the start index
					
						energy = 0;
						for(int j = 0; j < energyFrame; j++){ //compute evergy of frame
							energy = energy + ADCBuffer[i+j]*ADCBuffer[i+j];
						}
						
						if(energy > set_thresh+2000000){ //threshold detected...
								start = i;
								int pointer = i;
								
										for(int h = 0; h < 6*energyFrame; h++){ //When speech is detected, we go back 6 frames and fill speech up with those
											speech[6*energyFrame-fillCounter] = ADCBuffer[pointer];
											pointer--;
											if(pointer < 0){
												pointer = bufferSize-1;
											}
											fillCounter++;
										}
								
								for(int k = 0; k < energyFrame;k++){ //after back tracking 6 frames fill speech buffer with the remaining values in half buffer from the starting index
									speech[k+fillCounter] = ADCBuffer[i+k];
								}
								fillCounter = fillCounter + energyFrame;
						}
					
				}else if(fillCounter < speechSize){ //if speech has been previously detected we just need to make sure that we dont overflow speech buffer
					if(i == 0){
						for(int k = 0; k < energyFrame;k++){
						
							speech[k+fillCounter] = ADCBuffer[i+k];
						}
						fillCounter = fillCounter + energyFrame;
					}else{
						for(int k = 0; k < (energyFrame/2);k++){
						
							speech[k+fillCounter] = ADCBuffer[i+(energyFrame/2)+k];
						}
						fillCounter = fillCounter + (energyFrame/2);
					}
					
					if(fillCounter > speechSize-2){
						HAL_ADC_Stop_DMA(&hadc1); // Stop DMA for speech classification, need to restart it once number is recognized
						STOP_DMA = 1; //stop DMA flag used by the systick interrupt (will need to enventaully replace with Timer)
						fillCounter = 0; //reset fill counter for next speech to be captured
						speechCap = 1; //indicate that speech buffer has been filled
						start = -1; //reset starting index
						mahalaTrans(speech,speechSize);
						
						//Toggle LEDS to indicate that speech was captured
						HAL_GPIO_TogglePin(GPIOD, LED3_Pin);
						HAL_GPIO_TogglePin(GPIOD, LED4_Pin);
						HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
						//UART_Transmit_F(speech,speechSize); //for debugging purposes
						
						printf("UART DONE!!\n");
						

						for(int i = 0; i < 4000; i++){
							//buffer[i] = 32768*(buffer[i]);
							speech[i] = 32768*speech[i];
						}
						
						/* Pre-emphasis filter */
						for(int i = 1; i < 4000; i++){
							speech[i] = speech[i] - 0.95*speech[i-1];
						}
						
						mfccFunc(speech,mfcc);
						//UART_Transmit_F(mfcc,24*13);

						printf("mfcc DONE!!\n");
						
						classification(mfcc,results);
						UART_Transmit_R(results,10);
						printf("Classification DONE!!\n");
						
						//TODO: getting a hardfault at the end due to eithe space limitations or code implementation (USING CMSIS instead of my own fft and matrix multiplications may help)
						return 1;
					}
				}
				
			}
		
		}
		return 0;
}

//Mahalanobis Transform
void mahalaTrans(float *Speech, int size){
	float mean;
	for(int i = 0; i < size; i++){
		mean = mean + Speech[i];
	}
	mean = mean/size;
	float var;
	for(int i = 0; i < size; i++){
		var = var + pow((Speech[i]-mean),2);
	}
	var = var/size;
	for(int i = 0; i < size; i++){
		Speech[i] = (Speech[i]-mean)/var;
	}
	
	Speech[0] = Speech[2]; // first index is abnormally high, therefore we assign it a normal value
}

/* called when 1st half of buffer is filled */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* AdcHandle)
		{
			energyDetect(0);
		}

/* called when 2nd half of buffer is filled */
/* will increment callibration counter by 1 (every 0.5 seconds) */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
    {
			/* start DMA again using same buffer pointer (circular)*/ 
			HAL_ADC_Stop_DMA(&hadc1);
			HAL_ADC_Start_DMA(&hadc1,ADCBuffer,bufferSize);
			energyDetect(bufferSize/2);
			
			/* Will trigger when trheshold stage is complete */
			if(callibration < 4){
				callibration = callibration + 1;
			}
			if(callibration == 4){
				callibration = callibration + 1;
				HAL_GPIO_TogglePin(GPIOD, LD5_Pin); //Toggle LED to indicate that callibration is done
				float div = ((callibration-1)*2*98); //98 = (2000-40/20)
				set_thresh = set_thresh/div; //compute average for average energy
			}
    }

		
		
/* Function to send uint32_t buffer over UART */	
void UART_Transmit_U(uint32_t *array, int size)
		{
			char send[9];
			printf("Sending uint via UART...\n");
			for(int i = 0; i < size; i++){
					sprintf(send, "%lu,\r\n",array[i]);
					HAL_UART_Transmit(&huart2, send, 9, 1000);
			}
		}
		
/* Function to send float buffer over UART */	
void UART_Transmit_F(float *array, int size)
		{
			char send[20];
			printf("Sending float via UART...\n");
			for(int i = 0; i < size; i++){
					sprintf(send, " %f,\r\n",(float)array[i]);
					HAL_UART_Transmit(&huart2, send, 9, 1000);
					sprintf(send, "      \r\n");
					HAL_UART_Transmit(&huart2, send, 9, 1000);
					
			}
			printf("Done UART...\n");
		}

		
/* Function to send results from NN over UART */	
void UART_Transmit_R(float *array, int size)
		{
			char send[25];
			char send2[25];
			printf("Sending float via UART...\n");
					sprintf(send, "\nResults\r\n");
					HAL_UART_Transmit(&huart2, send, 10, 1000);
			for(int i = 0; i < size; i++){
				sprintf(send2,"   %d:    ",i);
					HAL_UART_Transmit(&huart2, send2, 10, 1000);
					HAL_UART_Transmit(&huart2, send2, 7, 1000);
					sprintf(send, "%f\r", array[i]);
					HAL_UART_Transmit(&huart2, send, 9, 1000);
					sprintf(send, "      \r\n");
					HAL_UART_Transmit(&huart2, send, 8, 1000);
					
			}
			printf("Done UART...\n");
		}		
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_ADC_Start(&hadc1);
	
	
	DMA_Active = 1;
	HAL_ADC_Start_DMA(&hadc1,ADCBuffer,bufferSize);
	
	
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/8000); //FORCING systick clock to be at 8kHz (TODO: trigger clock via timer instead of systick)

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED3_Pin|LED4_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED4_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
