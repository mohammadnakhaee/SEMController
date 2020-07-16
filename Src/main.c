/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/dhcp.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "tcp_echoserver.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;
DMA_HandleTypeDef hdma_dac2;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define bufnum 32
#define UDP_SERVER_PORT    23   /* define the UDP local connection port */
#define UDP_CLIENT_PORT    23  /* define the UDP remote connection port */
int wix;
int wiy;
int wnx=512;
int wny=512;
volatile int multiply=1,multiply_count=1,windowsize=1;
int wndata=1028;//514;
int nPrompt=1;
int iPrompt=0;
int nY=512;
char d[514];
int color;
uint16_t mainloop=1,mainloopp=1;
int16_t row;
int column;
char row1;
char row2;
struct udp_pcb *upcb;
volatile struct pbuf *buf[bufnum];
//volatile struct pbuf *buf2;
int time;
int Packet;
int nFramePrompt=512;
int nData=514;
struct netif gnetif;
volatile int isReadyToSend = 0 , tcp_command_recieved = 0 ,windowchanged=0;
volatile uint32_t xVoltageBuffer[512];
volatile uint32_t yVoltageBuffer[512];
char Row1Bytes[512];
char Row2Bytes[512];
volatile u8_t *MyPayload[bufnum];
int ADCBUFFER_SIZE=512;
volatile uint8_t ADCBuffer0[512];
volatile uint8_t ADCBuffer1[512];
volatile uint32_t Dac_x = 0;
volatile uint32_t Dac_y = 0;
int isDacTimerOn=0;
uint32_t ADC1_CHANNEL=ADC_CHANNEL_0;
uint32_t ADC1_SAMPLETIME=ADC_SAMPLETIME_3CYCLES;
uint8_t MuxSel0_2_data = 0;  //Right 4 bits of data
uint8_t MuxSel1_3_data = 0;  //Left 4 bits of data
//extern struct tcp_pcb *tcp_echoserver_pcb;
//extern struct tcp_echoserver_struct *ess;
extern char uRxBuffer[1];
extern volatile struct tcp_echoserver_struct *ans;
extern volatile struct pbuf *cammand;
int isSingleShot=0;
int isUart2ITMode=0;
int isUart6ITMode=0;
int isUart2=0;
int isUart6=0;
int isAcquire=0;
int AcquireCnt=0;
int AcquireNumber=0;
volatile char cc[26];
volatile int flag=0,flagtcp=0;
uint16_t iquad=0,dual=0;
int cci = 0;
struct tcp_echoserver_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};
volatile int LAN_is_Connected = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);

//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void udp_echoserver_init(void);
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void adc1_dma_callback0(DMA_HandleTypeDef *_hdma);
void adc1_dma_callback1(DMA_HandleTypeDef *_hdma);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//MX_GPIO_Init->pb0 dasti disable tim4 dasti pwm disable tim2 dasti /moosa change
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration----------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache-------------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LWIP_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */



  udp_echoserver_init();
  ip_addr_t addrr,localaddrr;
  IP4_ADDR(&addrr, 190, 100, 101, 2);
  IP4_ADDR(&localaddrr, 190, 100, 101, 1);

  /*
	for(int i=0; i<100; i++)
	{
	  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_Delay(10);
    }
*/
 ///////////
      GPIO_InitTypeDef GPIO_InitStruct;
      GPIO_InitStruct.Pin = GPIO_PIN_4;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      GPIO_InitStruct.Pin = GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 /////////////
  //int i;
  udp_bind(upcb, &localaddrr, UDP_CLIENT_PORT);
  udp_connect(upcb, &addrr, UDP_CLIENT_PORT);
  for(int i=0;i<bufnum;i++)
  buf[i] = pbuf_alloc(PBUF_RAW_TX, 514+42 ,PBUF_RAM );
 // buf2 = pbuf_alloc(PBUF_TRANSPORT, 514 , );PBUF_RAWPBUF_RAM


  uint8_t udp_h[42] =
    {
  		  0x34,0xe6,0xd7,0x05,0x59,0xfd,0x00,0x80, 0xe1,0x00,0x00,0x00,0x08,0x00,0x45,0x00, \
  		  0x02,0x1e,0x00,0x00,0x00,0x00,0xff,0x11, 0x00,0x00,0xbe,0x64,0x65,0x01,0xbe,0x64, \
  		  0x65,0x02,0x00,0x17,0x00,0x17,0x02,0x0a, 0x00,0x00
    };                                             //checksum //
    //udp_h[42] = {0x34,0xe6,0xd7,0x05,0x59,0xfd,0x00,0x80,0xe1,0x00,0x00,0x00,0x08,0x00,0x45,0x00,0x02,0x1e,0x00,0x00,0x00,0x00,0xff,0x11,0x73,0x02,0xbe,0x64,0x65,0x01,0xbe,0x64,0x65,0x02,0x00,0x17,0x00,0x17,0x02,0x0a,0x33,0x5f};

    //pbuf_take(buf, d, 512);
    uint16_t j;

    for(uint16_t i=0;i<bufnum;i++)
    {
    MyPayload[i] = (u8_t*)buf[i]->payload;// - (s16_t)(0);

  for(j=0;j<42;j++)
    *(MyPayload[i] + (s16_t)j) = (char)udp_h[j];

   MyPayload[i] += 42;
    }

   // HAL_TIM_Base_Start(&htim2);

    //HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
//while(1);
  row=0;
  column=0;
  time=0;
  Packet=0;

  TIM_SlaveConfigTypeDef sSlaveConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
/*
    htim5.Instance = TIM6;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 20000;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
      Error_Handler();
    }

    TIM_ClockConfigTypeDef sClockSourceConfig;

      sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
      if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
      {
        Error_Handler();
      }
   // sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  //  sSlaveConfig.InputTrigger = TIM_TS_ITR2;

 //   if (HAL_TIM_SlaveConfigSynchronization(&htim5, &sSlaveConfig) != HAL_OK)
    {
 //     Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    //sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }

*//*
  DAC_ChannelConfTypeDef sConfig;
    sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }*/
/*
  TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim2.Init.Period = 0;

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ITR3;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
      Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }
*/

 // if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 0x00) != HAL_OK) Error_Handler(); //Set DAC Channel1 DHR register
 // if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) Error_Handler(); // Enable DAC Channel1
  // HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, xVoltageBuffer, 512, DAC_ALIGN_12B_R);//dactest
    if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x00) != HAL_OK) Error_Handler(); //Set DAC Channel2 DHR register
      if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK) Error_Handler(); // Enable DAC Channel2
  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0x00) != HAL_OK) Error_Handler(); //Set DAC Channel2 DHR register
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_2) != HAL_OK) Error_Handler(); // Enable DAC Channel2
  //HaL_ADC_Start(;)
  //HAL_DMA_RegisterCallback(&hdma_adc1,HAL_DMA_XFER_CPLT_CB_ID, adc1_dma_callback);
  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) Error_Handler(); //Start Conversation Error
  hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 9U));//dds
  hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 10U));//EOCS
 // HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADCBuffer0, ADCBUFFER_SIZE);
  hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 9U));//dds
  hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 10U));//EOCS
  hadc1.Instance->CR2 |= ((uint32_t)(0x01U << 8U));//dma

  HAL_DMA_RegisterCallback(&hdma_adc1,HAL_DMA_XFER_CPLT_CB_ID  , adc1_dma_callback0);
  HAL_DMA_RegisterCallback(&hdma_adc1,HAL_DMA_XFER_M1CPLT_CB_ID, adc1_dma_callback1);
  /* Start the DMA channel */
     // HAL_DMA_Start_IT(hadc1->DMA_Handle, (uint32_t)&hadc1->Instance->DR, (uint32_t*)ADCBuffer0, ADC																																		BUFFER_SIZE);
//if (HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)&ADCBuffer0, ADCBUFFER_SIZE) != HAL_OK) Error_Handler();
HAL_DMAEx_MultiBufferStart_IT(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)(MyPayload[0]),(uint32_t)(MyPayload[1]), ADCBUFFER_SIZE);

  /* Disable end of conversion interrupt for regular group */
     // __HAL_ADC_DISABLE_IT(&hadc1, (ADC_IT_EOC | ADC_IT_OVR));
  /* Enable end of conversion interrupt for regular group */
 //     __HAL_ADC_ENABLE_IT(&hadc1, (ADC_IT_EOC | ADC_IT_OVR));
 //
  /* Disable Common interrupts*/
//  (&hdma_adc1)->Instance->CR  &= ~(DMA_IT_TC | DMA_IT_TE | DMA_IT_DME);
//  (&hdma_adc1)->Instance->FCR &= ~(DMA_IT_FE);
  /* Enable Common interrupts*/
//  (&hdma_adc1)->Instance->CR  |= DMA_IT_TC ;//| DMA_IT_TE | DMA_IT_DME;
//  (&hdma_adc1)->Instance->FCR |= 0;//DMA_IT_FE;


  /* tcp echo server Init */
  tcp_echoserver_init();
  /*tpcb = tcp_new();
  err_t err = tcp_bind(tpcb, IP_ADDR_ANY, 7);
  tpcb = tcp_listen(tpcb);
  tcp_accept(tpcb, tcp_echooo);*/


  ScanISel_Init();
  ScanTrim_Init();
  ScanZoomRot_Init();
  DetMuxSel_Init();
  DetAmpGain_Init();
  DetTrim_Init();



  /*struct pbuf *ptr;
  char dt[2];
  dt[0]=102;
  dt[1]=103;
  ptr = pbuf_alloc(PBUF_TRANSPORT, 2 , PBUF_REF);
  pbuf_take(ptr, dt, 2);
  es = (struct tcp_echoserver_struct *)mem_malloc(sizeof(struct tcp_echoserver_struct));
  es->p = ptr;*/

  dacrange(xVoltageBuffer,(uint16_t)2047, (uint16_t)2047,512);
  dacrange(yVoltageBuffer,(uint16_t)2047, (uint16_t)2047,512);





  for(int i=0; i<wny; i++)
  {
    Row1Bytes[i] = (char)i;
    Row2Bytes[i] = (char)(i >> 8);
    d[i]=0;
    //*(MyPayload[0] + i ) = (char)(i);
  }
  d[wnx] = 0;
  d[wnx+1] = 0;

//  HAL_Delay(2000);
 // udp_send(upcb, buf[0]);
//  HAL_Delay(2000);
//  udp_send(upcb, buf[0]);
 // udp_send(upcb, buf[0]);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  Dac_x = (uint32_t)hdac.Instance;
  Dac_x += DAC_DHR12R1_ALIGNMENT(DAC_ALIGN_12B_R);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  Dac_y = (uint32_t)hdac.Instance;
  Dac_y += DAC_DHR12R2_ALIGNMENT(DAC_ALIGN_12B_R);


  //ess = (struct tcp_echoserver_struct *)mem_malloc(sizeof(struct tcp_echoserver_struct));
  //tcp_write(tcp_echoserver_pcb, ptr->payload, ptr->len, 1);
//if (ptr)HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
//if (tcp_echoserver_pcb)HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
  //int old =ess->p->len;

//Initialize all spi pins
  //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);


  //ScanTrim((uint8_t)0, (uint16_t)3000);

  isDacTimerOn = 0;
 // HAL_TIM_Base_Stop_IT(&htim2);
  HAL_TIM_Base_Stop(&htim4);

  //First receive have problem
  //So we receive a char with a very short time out 10ms
  uint8_t receivedchar='\r';
  HAL_UART_Transmit(&huart2, &receivedchar, 1, 50);
  HAL_UART_Transmit(&huart6, &receivedchar, 1, 50);
  HAL_Delay(50);
  for (int i=0; i<50; i++)
  {
  HAL_UART_Receive(&huart2, &receivedchar, 1,10);
  HAL_UART_Receive(&huart6, &receivedchar, 1,10);
  }
  //////////////////////////////////////////////////////


  rotate(4095, 0); //theta = 0
  zoom(4095,4095); //end state of zoom back
  /* Clear the old sample time */
 // udp_send(upcb, buf[0]);
  //err_t wr_err;
  /* USER CODE END 2 */
//  HAL_TIM_Base_Start(&htim2);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (LAN_is_Connected == 0)
	  {
		  MX_LWIP_Init();
		  if (LAN_is_Connected == 1)
		  {
			  dhcp_start(&gnetif);
			  udp_echoserver_init();
			  udp_bind(upcb, &localaddrr, UDP_CLIENT_PORT);
			  udp_connect(upcb, &addrr, UDP_CLIENT_PORT);
			  tcp_echoserver_init();
		  }
	  }
	  //tcp_echoserver_send(tcp_echoserver_pcb, ess);
	  //HAL_Delay(1000);
	  //HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
	  //if (ess->p->len != old) HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
	  /*HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
      HAL_Delay(1000);
	  wr_err = tcp_write(tcp_echoserver_pcb, ptr->payload, ptr->len, 1);
	  tcp_output(tcp_echoserver_pcb);
	  //HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
	  if (wr_err != ERR_OK) HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
	  //HAL_Delay(10);*/
	  /*if(tcp_command_recieved)
	  {
		 HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
	     AnswerToOrder((char*)cammand->payload,ans);
	     tcp_echoserver_send(ans->pcb, ans);
	     tcp_command_recieved = 0;
	  }*/

if(mainloop++==0)
	if(mainloopp++==10)
	{
		//HAL_GPIO_TogglePin(GPIOB, LD1_Pin);//GPIO_PIN_14
    mainloopp=0;
	}
	  if(windowchanged != 0)
	  {

		  if(windowchanged==1)
		  windowset(0,0,0,0);
		  windowchanged=0;
		  if(isDacTimerOn == 1)
		  //HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
		  HAL_TIM_Base_Start(&htim4);
	  }


	  if (flag == 1)
	  {
		  //HAL_Delay(100);
		  flagtcp=1;
	  	 ans->p->payload = cc;
	  	 tcp_echoserver_send(ans->pcb, ans);
	  	 flag = 0;
	  	 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
	  }
	  else
	  {
		  if (isReadyToSend>=1)
		  {
		 // udp_send(upcb, buf[isReadyToSend-1]);
		/*	  if(dual==0)
			  memcpy(MyPayload[isReadyToSend-1],ADCBuffer1,512);
			  else
			  memcpy(MyPayload[isReadyToSend-1],ADCBuffer0,512);
*/
			  gnetif.linkoutput(&gnetif,buf[isReadyToSend-1]);
			  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
		  	  isReadyToSend = 0;
		  	//HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)&ADCBuffer, 512);
		  		     	 			/* Enable the Peripheral */
		  		     	 		   // __HAL_DMA_ENABLE(&hdma_adc1);
		  		     	 	//	    HAL_ADC_Start_DMA(&hadc1,  (uint32_t*)ADCBuffer, 512);//}//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, xVoltageBuffer, 512, DAC_ALIGN_12B_R);}
		  		     	 		  //__HAL_ADC_DISABLE_IT(&hadc1, (ADC_IT_EOC | ADC_IT_OVR));
		  		     	 	//	__HAL_TIM_ENABLE(&htim4);
		  }
	  }

	  MX_LWIP_Process();
	  //HAL_Delay(1000);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T4_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* ADC3 init function */
static void MX_ADC3_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_2TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CAN2 init function */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_2TQ;
  hcan2.Init.BS2 = CAN_BS2_2TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = DISABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

    /**DAC channel OUT2 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI4 init function */
static void MX_SPI4_Init(void)
{

  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  TIM_SlaveConfigTypeDef sSlaveConfig;
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;

      if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
      {
        Error_Handler();
      }

  //sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  //if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    //Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{
	 TIM_ClockConfigTypeDef sClockSourceConfig;
	  TIM_MasterConfigTypeDef sMasterConfig;

	  htim4.Instance = TIM4;
	    htim4.Init.Prescaler = 0;
	    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	    htim4.Init.Period = 200;
	    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	    {
	      Error_Handler();
	    }

	    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	    {
	      Error_Handler();
	    }

	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }


}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 1000000;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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
     PD8   ------> USART3_TX
     PD9   ------> USART3_RX
     PA8   ------> USB_OTG_FS_SOF
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG2 PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void udp_echoserver_init(void)
{
   err_t err;

   /* Create a new UDP control block  */
   upcb = udp_new();

   if (upcb)
   {
     /* Bind the upcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
      err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);

      if(err == ERR_OK)
      {
        /* Set a receive callback for the upcb */
        //udp_recv(upcb, udp_echoserver_receive_callback, NULL);
      }
      else
      {
        udp_remove(upcb);
      }
   }
}

void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
  /* Connect to the remote client */
  udp_connect(upcb, addr, UDP_CLIENT_PORT);

  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
  /* Tell the client that we have accepted it */
  udp_send(upcb, p);

  /* free the UDP connection, so we can accept new clients */
  udp_disconnect(upcb);

  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);
  /* Free the p buffer */
  pbuf_free(p);


}

void ScanISel(uint8_t data)
{
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	HAL_SPI_Init(&hspi3);

	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi3, &data, 1, 3000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
}

void ScanTrimij(uint8_t i, uint8_t j, uint16_t w)
{
	uint8_t dac = i*3+j;
	ScanTrim(dac, w);
}

void ScanTrim(uint8_t dac, uint16_t data)
{
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
	HAL_SPI_Init(&hspi3);

	uint8_t Buffer[2];
	uint16_t value;
	value = (0 << 15) | (dac << 12) | (data & 0x0fff);
	Buffer[0] = (uint8_t)(value >> 8);
	Buffer[1] = (uint8_t)value;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi3, Buffer, 2, 3000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);
}

void ScanZoomRot(uint8_t device, uint8_t dac, uint16_t data)
{
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
	HAL_SPI_Init(&hspi3);

	uint8_t d1=0;
	uint8_t d2=0;
	uint8_t d3=0;

	if (device==0) d1=1+3*dac;
	else if (device==1) d2=1+3*dac;
	else if (device==2) d3=1+3*dac;

	uint8_t Buffer[2];
	uint16_t value;
	value = (d3 << 12) | data;
	Buffer[0] = (uint8_t)(value >> 8);
	Buffer[1] = (uint8_t)value;
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi3, Buffer, 2, 3000);

	value = (d2 << 12) | data;
	Buffer[0] = (uint8_t)(value >> 8);
	Buffer[1] = (uint8_t)value;
	HAL_SPI_Transmit(&hspi3, Buffer, 2, 3000);

	value = (d1 << 12) | data;
	Buffer[0] = (uint8_t)(value >> 8);
	Buffer[1] = (uint8_t)value;
	HAL_SPI_Transmit(&hspi3, Buffer, 2, 3000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
}

void DetMuxSel(uint8_t data) //OK
{
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	HAL_SPI_Init(&hspi2);

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi2, &data, 1, 3000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
}

void DetAmpGain(uint8_t device, uint8_t Channel, uint8_t Gain)
{
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	HAL_SPI_Init(&hspi2);

	uint8_t d1=0;
	uint8_t d2=0;
	if (device==0) d1=2;
	else if (device==1) d2=2;

	uint8_t Buffer[4];
	uint8_t InstructionByte1=0;
	uint8_t InstructionByte2=0;
	InstructionByte1 = (d1 << 5) | 1; //write-0000-channel
	InstructionByte2 = (d2 << 5) | 1; //write-0000-channel

	Buffer[0] = InstructionByte2;
	Buffer[1] = Channel;
	Buffer[2] = InstructionByte1;
	Buffer[3] = Channel;

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi2, Buffer, 4, 3000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);

	HAL_Delay(1);
	InstructionByte1 = 0;
	InstructionByte2 = 0;
	InstructionByte1 = (d1 << 5) | 0; //write-0000-gain
	InstructionByte2 = (d2 << 5) | 0; //write-0000-gain

	Buffer[0] = InstructionByte2;
	Buffer[1] = Gain;
	Buffer[2] = InstructionByte1;
	Buffer[3] = Gain;

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi2, Buffer, 4, 3000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
}

void DetTrimij(uint8_t i, uint8_t j, uint16_t w)
{
	uint8_t dac = i*3+j;
	DetTrim(dac, w);
}

void DetTrim(uint8_t dac, uint16_t data)
{
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	HAL_SPI_Init(&hspi2);

	uint8_t Buffer[2];
	uint16_t value;
	value = (0 << 15) | (dac << 12) | (data & 0x0fff);
	Buffer[0] = (uint8_t)(value >> 8);
	Buffer[1] = (uint8_t)value;
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi2, Buffer, 2, 3000);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
}

void ScanISel_Init()
{
	ScanISel((uint8_t)0);
}

void ScanTrim_Init()
{
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
	HAL_SPI_Init(&hspi3);
}

void ScanZoomRot_Init()
{
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
	HAL_SPI_Init(&hspi3);
}

void DetMuxSel_Init()
{
	MuxSel0_2_data = 0;
	MuxSel1_3_data = 0;
	uint8_t data = (MuxSel1_3_data << 4) | (MuxSel0_2_data << 4);
	DetMuxSel(data);
}

void DetAmpGain_Init()
{
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	HAL_SPI_Init(&hspi2);

}

void DetTrim_Init()
{
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	HAL_SPI_Init(&hspi2);
}

void sendhello2(char* aRxBuffer)
{
	uint8_t aTxBuffer[] = "1";
	aRxBuffer[0] = '0';
	aRxBuffer[1] = '\r';
	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
	if(HAL_UART_Transmit(&huart2, (uint8_t*)aTxBuffer, 1, 3000)== HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
	}
	if(HAL_UART_Receive(&huart2, (uint8_t *)aRxBuffer, 1,3000)== HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
	}
}

void sendhello6(char* aRxBuffer)
{
	uint8_t aTxBuffer[] = "1";
	aRxBuffer[0] = '0';
	aRxBuffer[1] = '\r';
	HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
	if(HAL_UART_Transmit(&huart6, (uint8_t*)aTxBuffer, 1, 3000)== HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
	}
	if(HAL_UART_Receive(&huart6, (uint8_t *)aRxBuffer, 1,3000)== HAL_OK)
	{
		HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
	}
}

void dacrange(uint32_t* VoltageBuffer, uint16_t meanval, uint16_t amp,uint16_t size)
{
  if (isDacTimerOn == 1) HAL_TIM_Base_Stop_IT(&htim2);
  uint16_t Voltage0 = meanval;
  uint16_t VoltageAmp = amp;
  if (VoltageAmp > Voltage0) VoltageAmp=Voltage0;
  if (VoltageAmp > 4095 - Voltage0) VoltageAmp=4095 - Voltage0;
  if (VoltageAmp < 0) VoltageAmp=0;
  uint16_t VoltageMin = Voltage0 - VoltageAmp;
  float dVoltage = 2.0*VoltageAmp/(size - 1.0);

  for (int i=0; i<size; i++)
  {
	  VoltageBuffer[i] = VoltageMin + dVoltage*i;
	  if (VoltageBuffer[i] > 4095) VoltageBuffer[i]=4095;
	  if (VoltageBuffer[i] < 0) VoltageBuffer[i]=0;
  }
  if (isDacTimerOn == 1) HAL_TIM_Base_Start_IT(&htim2);
}

void SetADC1Channel(uint32_t ADC_CHANNEL,uint32_t ADC_SAMPLETIME)
{
	  ADC_ChannelConfTypeDef sConfig;
	  sConfig.Channel = ADC_CHANNEL;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

int setsignal(uint8_t Port, uint8_t terminalkind, uint8_t terminal, uint8_t gain)
{
	//Port  0,1,2,3
	//terminalkind [direct 0],[MCP 1],[ADG 2] if port = 0,1 ([MCP 1],[ADG 2] if port = 2,3)
	//terminal 0,1,2,3 if terminalkind != 0
	//gain 12bit
  int isValid=0;

  if (Port == 0)
  {
	  if (terminalkind == 0) //Direct
	  {
		  ADC1_CHANNEL = ADC_CHANNEL_6; //PA6 is ADC_IN6
		  SetADC1Channel(ADC1_CHANNEL,ADC1_SAMPLETIME);
		  isValid=1;
	  }
	  else if (terminalkind == 1) //MCP
	  {
		  ADC1_CHANNEL = ADC_CHANNEL_0; //PA0 is ADC_IN0
		  SetADC1Channel(ADC1_CHANNEL,ADC1_SAMPLETIME);
		  DetAmpGain(0, terminal, gain);
		  isValid=1;
	  }
	  else if (terminalkind == 2) //ADG
	  {
		  ADC1_CHANNEL = ADC_CHANNEL_3; //PA3 is ADC_IN3
		  SetADC1Channel(ADC1_CHANNEL,ADC1_SAMPLETIME);
		  uint8_t MuxSel0_2_data = 0;
		  MuxSel0_2_data = (1 << 3) | terminal;
		  uint8_t data = (MuxSel1_3_data << 4) | MuxSel0_2_data;
		  DetMuxSel(data);
		  isValid=1;
	  }
  }
  else if (Port == 1)
  {
	  if (terminalkind == 0) //Direct
	  {
		  ADC1_CHANNEL = ADC_CHANNEL_9; //PB1 is ADC_IN9
		  SetADC1Channel(ADC1_CHANNEL,ADC1_SAMPLETIME);
		  isValid=1;
	  }
	  else if (terminalkind == 1) //MCP
	  {
		  ADC1_CHANNEL = ADC_CHANNEL_10; //PC0 is ADC_IN10
		  SetADC1Channel(ADC1_CHANNEL,ADC1_SAMPLETIME);
		  DetAmpGain(1, terminal, gain);
		  isValid=1;
	  }
	  else if (terminalkind == 2) //ADG
	  {
		  ADC1_CHANNEL = ADC_CHANNEL_13; //PC3 is ADC_IN13
		  SetADC1Channel(ADC1_CHANNEL,ADC1_SAMPLETIME);
		  uint8_t MuxSel1_3_data = 0;
		  MuxSel0_2_data = (1 << 3) | terminal;
		  uint8_t data = (MuxSel1_3_data << 4) | MuxSel0_2_data;
		  DetMuxSel(terminal);
		  isValid=1;
	  }
  }
  else if (Port == 2)
  {
	  if (terminalkind == 1) //MCP
	  {
		  ADC1_CHANNEL = ADC_CHANNEL_0; //PA0 is ADC_IN0
		  SetADC1Channel(ADC1_CHANNEL,ADC1_SAMPLETIME);
		  DetAmpGain(0, terminal, gain);
		  isValid=1;
	  }
	  else if (terminalkind == 2) //ADG
	  {
		  ADC1_CHANNEL = ADC_CHANNEL_3; //PA3 is ADC_IN3
		  SetADC1Channel(ADC1_CHANNEL,ADC1_SAMPLETIME);
		  uint8_t MuxSel0_2_data = 0;
		  MuxSel0_2_data = (1 << 3) | terminal;
		  uint8_t data = (MuxSel1_3_data << 4) | MuxSel0_2_data;
		  DetMuxSel(terminal);
		  isValid=1;
	  }
  }
  else if (Port == 3)
  {
	  if (terminalkind == 1) //MCP
	  {
		  ADC1_CHANNEL = ADC_CHANNEL_10; //PC0 is ADC_IN10
		  SetADC1Channel(ADC1_CHANNEL,ADC1_SAMPLETIME);
		  DetAmpGain(1, terminal, gain);
		  isValid=1;
	  }
	  else if (terminalkind == 2) //ADG
	  {
		  ADC1_CHANNEL = ADC_CHANNEL_13; //PC3 is ADC_IN13
		  SetADC1Channel(ADC1_CHANNEL,ADC1_SAMPLETIME);
		  uint8_t MuxSel1_3_data = 0;
		  MuxSel0_2_data = (1 << 3) | terminal;
		  uint8_t data = (MuxSel1_3_data << 4) | MuxSel0_2_data;
		  DetMuxSel(terminal);
		  isValid=1;
	  }
  }

  return isValid;
}

void rotate(uint16_t a00, uint16_t a10)
{
	ScanZoomRot(0, 0, a00);
	ScanZoomRot(1, 0, a10);
	ScanZoomRot(0, 1, a00);
	ScanZoomRot(1, 1, a10);
}

void zoom(uint16_t cx, uint16_t cy)
{
	ScanZoomRot(2, 0, cx);
	ScanZoomRot(2, 1, cy);
}

void FreeReceiveBuf2()
{
	uint8_t receivedchar;
	for (int i=0; i<26; i++)
	{
		HAL_UART_Receive(&huart2, &receivedchar, 1,1);
	}
}

void FreeReceiveBuf6()
{
	uint8_t receivedchar;
	for (int i=0; i<26; i++)
	{
		HAL_UART_Receive(&huart6, &receivedchar, 1,1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	//if ((isUart6 == 1 && isUart6ITMode == 1))
	//{
		if (flag != 1)
		{
			if (uRxBuffer[0]=='\r')
			{
				if (cci > 0)
				{
					cc[cci] = '\r';
					flag = 1;
					cci=0;
					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
				}
			}
			else
			{
			    cc[cci++] = uRxBuffer[0];
			}
		}
		HAL_UART_Receive_IT(UartHandle, (uint8_t*)uRxBuffer, 1);
	//}
  /* Turn LED4 on: Transfer in reception process is correct */

    //HAL_Delay(1);
    //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
}
void HAL_USART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	for(int i=0; i<100; i++)
	{
	  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_Delay(10);
	  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_Delay(10);
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

	//*(uint32_t *) Dac_y = xVoltageBuffer[(column++)];
	//if(column>=win) column=0;
	/*return;
	 if (isReadyToSend==0)
		 {
		if(flagtcp==0)
		 {

		 *(uint32_t *) Dac_x = xVoltageBuffer[column + wix];
		 	    if (column == 0)
		    {
		 	    	 // if (isSingleShot == 0)
		 	  *(uint32_t *) Dac_y = yVoltageBuffer[row + wiy];

		    }



		       // ADCVoltage = HAL_ADC_GetValue(&hadc1);
		     //	*(MyPayload  + (wnx+2) * (iquad) + column) = ADCVoltage;
		        //if(iquad==0)
		 	   *(MyPayload[iquad]  +  column) = HAL_ADC_GetValue(&hadc1);
		       // else
		        //*(MyPayload2  +  column) = ADCVoltage;
	            column++;
		        if (column > (wnx - 1))
		        {
		        	column = 0;
		        	 iquad = row-bufnum*(row/bufnum);
		        		  	  //*(MyPayload + (wnx+2) * (iquad) + wnx) = Row2Bytes[row];
		        		  	  //*(MyPayload + (wnx+2) * (iquad) + wnx + 1) = Row1Bytes[row];
		        	// if(iquad==0)
		        	 {
		        	*(MyPayload[iquad] + wnx) = Row2Bytes[row];
		        	*(MyPayload[iquad] + wnx + 1) = Row1Bytes[row];
		        	 }
		        	// else
		        	 //{
		        	// *(MyPayload2 + wnx) = Row2Bytes[row];
		        	// *(MyPayload2 + wnx + 1) = Row1Bytes[row];
		        	 //}
		        	         // quad+=514; if(quad>2056) iquad=0;
		        		  	  row++;
		        		  	  if (row > (wny - 1))
		        		      {
		        		  		  row = 0;
		        		  		  if (isAcquire == 1)
		        		  		  {
		        		  		      AcquireCnt++;
		        		  			  if (AcquireCnt == AcquireNumber)
		        		  			  {
		        		  				AcquireCnt=0;
		        		  				isAcquire = 0;
		        		  				isDacTimerOn = 0;
		        		  				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
		        		  				HAL_TIM_Base_Stop_IT(&htim2);
		        		  			  }
		        		  		  }

		        		  	  }
		        		  	//  if(iquad == 1)
		        		  	  isReadyToSend = iquad + 1;
		        		 // 	if(iquad==0)
		        		  //	udp_send(upcb, buf[iquad]);
		        		 // 	else
		        		 // 	udp_send(upcb, buf2);
		        }
		 }
		 }
		// else
		 {
			 //udp_send(upcb, buf[isReadyToSend-1]);
			// isReadyToSend = 0;

		 }*/
}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	//HAL_TIM_Base_Stop(&htim4);
	//__HAL_TIM_DISABLE(&htim4);
	htim4.Instance->CNT=1;
	/* Clear ADC overrun flag */
	    __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
	    //hadc->Instance->CR1
//	HAL_Delay(1);
	// Clear regular group conversion flag
//	__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_STRT | ADC_FLAG_EOC);
//	__HAL_TIM_ENABLE(&htim4);

}
 void adc1_dma_callback0(DMA_HandleTypeDef *_hdma)
 {
	 uint16_t nextquad;

	 nextquad = iquad + 2;
	 if( nextquad >= bufnum ) nextquad = nextquad - bufnum;
	// while(isReadyToSend==0);
		//  isReadyToSend=0;
	// column=0;
	      //   HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
	        _hdma->Instance->M0AR = MyPayload[nextquad];
		    *(MyPayload[iquad] + wnx) = Row2Bytes[row];
		    *(MyPayload[iquad] + wnx +1) = Row1Bytes[row];

	 if((multiply_count++)>=multiply)
	 {

	        *(uint32_t *) Dac_y = yVoltageBuffer[row + wiy];

		    multiply_count=1;
		   //
		    //HAL_DMAEx_ChangeMemory();
		      row++;column=wix*multiply;
		      if (row >= (wny) )
		      		        		      {
		      		        		  		  row = 0;
		      		        		  		 // if (isAcquire == 1)
		      		        		  		  {
		      		        		  		    /*  AcquireCnt++;
		      		        		  			  if (AcquireCnt == AcquireNumber)
		      		        		  			  {
		      		        		  				AcquireCnt=0;
		      		        		  				isAcquire = 0;
		      		        		  				isDacTimerOn = 0;
		      		        		  				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
		      		        		  				HAL_TIM_Base_Stop_IT(&htim2);
		      		        		  			  }*/
		      		        		  		  }

		      		        		  	  }
	 }
		 //     if (row <= 511)
		 //     isReadyToSend += 0;
		//      else row=0;
		//      *(uint32_t *) Dac_y = yVoltageBuffer[row];



		   //   dual++; if(dual>=2) dual=0;
	          iquad++;
		      isReadyToSend=(iquad);
		       if(iquad>=bufnum) iquad=0;
		     	 		//  udp_send(upcb, buf);  MX_LWIP_Process();
		      /* Configure the source, destination address and the data length */
		        //  DMA_SetConfig((&hdma_adc1), SrcAddress, DstAddress, DataLength);
		     	 	//	if (row <= 33000)
		     	 		{
		     	 			//HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)&ADCBuffer, 512);
		     	 		//	hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 9U));//dds
		     	 			hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 8U));//dds
		     	 			hadc1.Instance->CR2 |=  ((uint32_t)(0x01U << 8U));//dds
		     	 			/* Enable the Peripheral */
		     	 		   // __HAL_DMA_ENABLE(&hdma_adc1);
		     	 		//	if(dual==0)
		     	 		  //  HAL_ADC_Start_DMA(&hadc1,  (uint32_t*)ADCBuffer0, wnx);
		     	 		//	HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)&ADCBuffer0, wnx);
		     	 		//	else
		     	 			//HAL_ADC_Start_DMA(&hadc1,  (uint32_t*)ADCBuffer0, wnx);
		     	 		//	HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)&ADCBuffer1, wnx);
		     	 		  /* Start the DMA channel */
		     	 		    //  HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)hadc1.Instance->DR, (uint32_t)&MyPayload[0], 512);
		     	 		    //}//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, xVoltageBuffer, 512, DAC_ALIGN_12B_R);}
		     	 		  //__HAL_ADC_DISABLE_IT(&hadc1, (ADC_IT_EOC | ADC_IT_OVR));
		     	 		}
		     	 //		else row=0;

		     	 		//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
 }
 void adc1_dma_callback1(DMA_HandleTypeDef *_hdma)
  {
	 uint16_t nextquad;

	 nextquad = iquad + 2;
	 if( nextquad >= bufnum ) nextquad = nextquad - bufnum;
	// while(isReadyToSend==0);
		//  isReadyToSend=0;
	// column=0;
	      //   HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);_
	        _hdma->Instance->M1AR = MyPayload[nextquad];
		    *(MyPayload[iquad] + wnx) = Row2Bytes[row];
		    *(MyPayload[iquad] + wnx +1) = Row1Bytes[row];
	 if((multiply_count++)>=multiply)
	 {

	        *(uint32_t *) Dac_y = yVoltageBuffer[row + wiy];

		    multiply_count=1;
		   //
		    //HAL_DMAEx_ChangeMemory();
		      row++;column=wix*multiply;
		      if (row >= (wny) )
		      		        		      {
		      		        		  		  row = 0;
		      		        		  		 // if (isAcquire == 1)
		      		        		  		  {
		      		        		  		    /*  AcquireCnt++;
		      		        		  			  if (AcquireCnt == AcquireNumber)
		      		        		  			  {
		      		        		  				AcquireCnt=0;
		      		        		  				isAcquire = 0;
		      		        		  				isDacTimerOn = 0;
		      		        		  				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
		      		        		  				HAL_TIM_Base_Stop_IT(&htim2);
		      		        		  			  }*/
		      		        		  		  }

		      		        		  	  }
	 }
		 //     if (row <= 511)
		 //     isReadyToSend += 0;
		//      else row=0;
		//      *(uint32_t *) Dac_y = yVoltageBuffer[row];



		   //   dual++; if(dual>=2) dual=0;
	          iquad++;
		      isReadyToSend=(iquad);
		       if(iquad>=bufnum) iquad=0;
		     	 		//  udp_send(upcb, buf);  MX_LWIP_Process();
		      /* Configure the source, destination address and the data length */
		        //  DMA_SetConfig((&hdma_adc1), SrcAddress, DstAddress, DataLength);
		     	 	//	if (row <= 33000)
		     	 		{
		     	 			//HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)&ADCBuffer, 512);
		     	 		//	hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 9U));//dds
		     	 			hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 8U));//dds
		     	 			hadc1.Instance->CR2 |=  ((uint32_t)(0x01U << 8U));//dds
		     	 			/* Enable the Peripheral */
		     	 		   // __HAL_DMA_ENABLE(&hdma_adc1);
		     	 		//	if(dual==0)
		     	 		  //  HAL_ADC_Start_DMA(&hadc1,  (uint32_t*)ADCBuffer0, wnx);
		     	 		//	HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)&ADCBuffer0, wnx);
		     	 		//	else
		     	 			//HAL_ADC_Start_DMA(&hadc1,  (uint32_t*)ADCBuffer0, wnx);
		     	 		//	HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)&ADCBuffer1, wnx);
		     	 		  /* Start the DMA channel */
		     	 		    //  HAL_DMA_Start_IT(&hdma_adc1, (uint32_t)hadc1.Instance->DR, (uint32_t)&MyPayload[0], 512);
		     	 		    //}//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, xVoltageBuffer, 512, DAC_ALIGN_12B_R);}
		     	 		  //__HAL_ADC_DISABLE_IT(&hadc1, (ADC_IT_EOC | ADC_IT_OVR));
		     	 		}
		     	 //		else row=0;

		     	 		//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
  }
 void windowset(int ix,int iy,int nx,int ny)
 {
	// if(isDacTimerOn == 1)
	 			//HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
	 			//__HAL_TIM_DISABLE(&htim4);
	 //htim4.Instance->CR1 &= ~(TIM_CR1_CEN);
	 	 			//HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1); dactest

	 	 			//__HAL_DMA_DISABLE_IT(&hdma_adc1);
	 	 			//__HAL_DMA_DISABLE_IT(&hdma_adc1);
	 	 			HAL_DMA_Abort_IT(&hdma_adc1);
	 	 			htim4.Instance->CNT=1;
	 	 			row=0;
	 	 			multiply_count = 1;
	 	 		//	wix = ix;
	 	 		//    wiy = iy;
	 	 		//	wnx = nx;
	 	 		//	wny = ny;
	 	 			hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 8U));//dds
	 	 			hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 9U));//dds
	 	 			hadc1.Instance->CR2 &= ~((uint32_t)(0x01U << 10U));//EOCS
	 	 			hadc1.Instance->CR2 |= ((uint32_t)(0x01U << 8U));//dma
	 	 			//HAL_DMA_RegisterCallback(&hdma_adc1,HAL_DMA_XFER_CPLT_CB_ID  , adc1_dma_callback0);
	 	 			//HAL_DMA_RegisterCallback(&hdma_adc1,HAL_DMA_XFER_M1CPLT_CB_ID, adc1_dma_callback1);

	 	 			//HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, xVoltageBuffer+(wix), wnx, DAC_ALIGN_12B_R);//dactest
	 	 			column=wix*multiply;

	 	 			HAL_DMAEx_MultiBufferStart_IT(&hdma_adc1, (uint32_t)&(hadc1.Instance->DR), (uint32_t)(MyPayload[0]),(uint32_t)(MyPayload[1]), wnx);

 }


//int setsignal(uint8_t Port, uint8_t terminalkind, uint8_t terminal, uint16_t gain)
//Detectors
//Indirect Ports
// Signal 0  <---> Port {0 or 2}  <--->  MCP 0  <---> PA0
// Signal 1  <---> Port {1 or 3}  <--->  MCP 1  <---> PC0
// Signal 2  <---> Port {0 or 2}  <--->  ADG 508  <---> PA3
// Signal 3  <---> Port {1 or 3}  <--->  ADG 508  <---> PC3
//Direct Ports
// Signal 4 :: Port {0}  <---> PA6 (PB0)
// Signal 5 :: Port {1}  <---> PB1
//

//First board
//
//scanner  board 74hc595 spi3 selection port f1 should be 0 (Scanner Current selector) Command:iscan Range:0,1,2
//scanner  board ad5328  spi3 selection port f2 should be 0 (Scanner Trimer selector) Command:(a:xpos b:xfin c:ypos d:yfin e:xref g:yref) Range:0,1,2,3,4,5,6,7
//scanner  board ad5449  spi3 selection port f0 should be 0 [dazy chain 3] (zoom and rotation) Command:(zr i,j,w) i=0:1(x,y) j=0:2 w=word
//detector board 74hc595 spi2 selection port g1 should be 0 (Detector mux selector) Command:mux Range:0,1,2,3,4,5,6,7
//detector board mcp6s28 spi2 selection port g0 should be 0 [dazy chain 2] (Detector gain) Command:(detgain i,j,w) i=0:1 j=0:3 w=word
//detector board ad5328  spi2 selection port g2 should be 0 (Detector Trimer selector) Command:(detoffset i,j,w) i=0:1 j=0:3 w=word
//
//And also spi4 , spi5 should be just configured to be used later
//
//detector board det0 spi4 selection port e2 should be 0
//detector board det1 spi5 selection port e3 should be 0
//detector board det2 spi4 selection port e4 should be 0
//detector board det3 spi5 selection port e5 should be 0
//
//Start UArt2=port1 and UArt6=port2
//
//Can1 and Can2
//
//spi3 (clk: c10   data output(Master out slave in)(MOSI): C12) [data input(Master in slave out)(MISO): C11]
//chip select F2
//


/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disables the MPU */
  HAL_MPU_Disable();
    /**Initializes and configures the Region and the memory to be protected 
    */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
