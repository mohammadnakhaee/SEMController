/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"

/* USER CODE BEGIN 0 */
#include "lwip.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

extern int wix;
extern int wiy;
extern int wnx;
extern int wny;
extern int nPrompt;
extern int iPrompt;
extern int nY;
extern int color;
extern int16_t row;
extern int column;
extern char row1;
extern char row2;
extern struct udp_pcb *upcb;
//extern volatile struct pbuf *buf[bufnum];
extern volatile struct pbuf *buf2;
extern int time;
extern int Packet;
extern int nFramePrompt;
uint32_t xDACVoltage=0;
uint32_t yDACVoltage=0;
uint32_t ADCVoltage=0;
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac;
extern volatile int isReadyToSend;
extern uint32_t xVoltageBuffer[512];
extern uint32_t yVoltageBuffer[512];
extern char Row1Bytes[512];
extern char Row2Bytes[512];
extern volatile u8_t *MyPayload[32];
extern volatile uint8_t ADCBuffer0[512];
extern volatile uint8_t ADCBuffer1[512];
extern volatile uint32_t Dac_x;
extern volatile uint32_t Dac_y;
extern int isSingleShot;
extern int isAcquire;
extern int AcquireCnt;
extern int AcquireNumber;
extern int isDacTimerOn;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
//int cci = 0;
extern volatile char cc[26];
extern volatile int flag;
extern int isUart2ITMode;
extern int isUart6ITMode;
extern int isUart2;
extern int isUart6;
extern int SSDotNum;
extern int SSiDot;
uint8_t dot=0;
extern uint16_t iquad,dual;
extern volatile int flagtcp;
extern TIM_HandleTypeDef htim4;
extern volatile int multiply,multiply_count,windowsize;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ETH_HandleTypeDef heth;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_dac1;
extern DMA_HandleTypeDef hdma_dac2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 stream5 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac1);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream6 global interrupt.
*/
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac2);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
* @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
*/
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */
	//*(uint32_t *) Dac_x = xVoltageBuffer[(column) + wix];
	*(uint32_t *) Dac_x = xVoltageBuffer[(column++)/multiply];
	//	if ((column++) > (wnx - 1))
	 //   column = 0;
  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
 // HAL_ADC_IRQHandler(&hadc2);
 // HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	/*if ((isUart2 == 1 && isUart2ITMode == 1))
	{
		uint8_t c;
		c = (uint8_t)((&huart6)->Instance->RDR);
		if (flag != 1)
		{
			if (!(c=='\r' || cci==5))
			{
				cc[cci++] = c;
			}
			else
			{
				if (cci > 0)
				{
					cc[cci] = 0;
					flag = 1;
					cci=0;
				}
			}
		}
	}*/
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */
	//__HAL_TIM_DISABLE(&htim4);
	//hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 9U));//dds
  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
* @brief This function handles Ethernet global interrupt.
*/
void ETH_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_IRQn 0 */

  /* USER CODE END ETH_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_IRQn 1 */

  /* USER CODE END ETH_IRQn 1 */
}

/**
* @brief This function handles Ethernet wake-up interrupt through EXTI line 19.
*/
void ETH_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN ETH_WKUP_IRQn 0 */

  /* USER CODE END ETH_WKUP_IRQn 0 */
  HAL_ETH_IRQHandler(&heth);
  /* USER CODE BEGIN ETH_WKUP_IRQn 1 */

  /* USER CODE END ETH_WKUP_IRQn 1 */
}

/**
* @brief This function handles USART6 global interrupt.
*/
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
	//flag = 1;
/*if ((isUart6 == 1 && isUart6ITMode == 1))
{

	uint8_t c='n';
	c = *((&huart6)->pRxBuffPtr);
	//HAL_UART_Receive_IT(&huart6, &c, 1);
	if (flag != 1)
	{
		if (!(c=='\r' || cci==25))
		{
			cc[cci++] = c;
		}
		else
		{
			if (cci > 0)
			{
				cc[cci] = 0;
				flag = 1;
				cci=0;
			}
		}
	}
}*/
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */



/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
