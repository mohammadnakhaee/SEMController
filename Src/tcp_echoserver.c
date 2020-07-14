/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of and a contribution to the lwIP TCP/IP stack.
 *
 * Credits go to Adam Dunkels (and the current maintainers) of this software.
 *
 * Christiaan Simons rewrote this file to get a more stable echo application.
 *
 **/

 /* This file was modified by ST */

#include "tcp_echoserver.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include <stdlib.h>
#include <string.h>
#include <main.h>

#if LWIP_TCP

struct tcp_pcb *tcp_echoserver_pcb;
volatile struct tcp_echoserver_struct *ans;
volatile struct pbuf *cammand;
extern volatile int tcp_command_recieved,windowchanged;
extern volatile int multiply,multiply_count,windowsize;
char aRxBuffer[26]; //26 is defined in .net app because of TCP.
char aTxBuffer[26];//26 is defined in .net app because of TCP.
char uRxBuffer[1];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern DAC_HandleTypeDef hdac;
extern ADC_HandleTypeDef hadc1;
extern int isDacTimerOn;
extern volatile uint32_t xVoltageBuffer[512];
extern volatile uint32_t yVoltageBuffer[512];
extern int isSingleShot;
extern int isUart2ITMode;
extern int isUart6ITMode;
extern int isUart2;
extern int isUart6;
extern int isAcquire;
extern int AcquireCnt;
extern int AcquireNumber;
extern int16_t row;
extern int column;
extern int wix;
extern int wiy;
extern int wnx;
extern int wny;
extern int wndata;
extern volatile struct pbuf *buf;
extern volatile uint32_t Dac_x;
extern volatile uint32_t Dac_y;
extern volatile int flagtcp;
int SSDotNum = 0;
int SSiDot = 0;
/* ECHO protocol states */
enum tcp_echoserver_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

/* structure for maintaing connection infos to be passed as argument 
   to LwIP callbacks*/
struct tcp_echoserver_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};


err_t tcp_echoserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
err_t tcp_echoserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
void tcp_echoserver_error(void *arg, err_t err);
err_t tcp_echoserver_poll(void *arg, struct tcp_pcb *tpcb);
err_t tcp_echoserver_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
void tcp_echoserver_send(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es);
void tcp_echoserver_connection_close(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es);
void AnswerToOrder(char* Command,struct tcp_echoserver_struct *es);
int check(char* order,char* word);
int GetInputs(char* Command, int* inputs);
int GetChildCommand(char* Command, char* ChildCommand);
int IsSame(char* s1,char* s2, int n);

/**
  * @brief  Initializes the tcp echo server
  * @param  None
  * @retval None
  */
void tcp_echoserver_init(void)
{
  /* create new tcp pcb */
  tcp_echoserver_pcb = tcp_new();

  if (tcp_echoserver_pcb != NULL)
  {
    err_t err;
    
    /* bind echo_pcb to port 7 (ECHO protocol) */
    err = tcp_bind(tcp_echoserver_pcb, IP_ADDR_ANY, 152);
    
    if (err == ERR_OK)
    {
      /* start tcp listening for echo_pcb */
      tcp_echoserver_pcb = tcp_listen(tcp_echoserver_pcb);
      
      /* initialize LwIP tcp_accept callback function */
      tcp_accept(tcp_echoserver_pcb, tcp_echoserver_accept);
    }
    else
    {
      /* deallocate the pcb */
      memp_free(MEMP_TCP_PCB, tcp_echoserver_pcb);
    }
  }
}

/**
  * @brief  This function is the implementation of tcp_accept LwIP callback
  * @param  arg: not used
  * @param  newpcb: pointer on tcp_pcb struct for the newly created tcp connection
  * @param  err: not used 
  * @retval err_t: error status
  */
err_t tcp_echoserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  /* set priority for the newly accepted tcp connection newpcb */
  tcp_setprio(newpcb, TCP_PRIO_MIN);

  /* allocate structure es to maintain tcp connection informations */
  es = (struct tcp_echoserver_struct *)mem_malloc(sizeof(struct tcp_echoserver_struct));
  if (es != NULL)
  {
    es->state = ES_ACCEPTED;
    es->pcb = newpcb;
    es->retries = 0;
    es->p = NULL;
    
    /* pass newly allocated es structure as argument to newpcb */
    tcp_arg(newpcb, es);
    
    /* initialize lwip tcp_recv callback function for newpcb  */ 
    tcp_recv(newpcb, tcp_echoserver_recv);
    
    /* initialize lwip tcp_err callback function for newpcb  */
    tcp_err(newpcb, tcp_echoserver_error);
    
    /* initialize lwip tcp_poll callback function for newpcb */
    tcp_poll(newpcb, tcp_echoserver_poll, 0);
    //moosa
    tcp_sent(newpcb, tcp_echoserver_sent);
    //moosa end
    //HAL_GPIO_TogglePin(GPIOB, LD1_Pin);

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);

    ret_err = ERR_OK;
  }
  else
  {
    /*  close tcp connection */
    tcp_echoserver_connection_close(newpcb, es);
    /* return memory error */
    ret_err = ERR_MEM;
  }

  return ret_err;  
}


/**
  * @brief  This function is the implementation for tcp_recv LwIP callback
  * @param  arg: pointer on a argument for the tcp_pcb connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  pbuf: pointer on the received pbuf
  * @param  err: error information regarding the reveived pbuf
  * @retval err_t: error code
  */
err_t tcp_echoserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_echoserver_struct *es;
  err_t ret_err;
//HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
HAL_GPIO_TogglePin(GPIOB, LD1_Pin);
  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct tcp_echoserver_struct *)arg;
  ans=es;
  /* if we receive an empty tcp frame from client => close connection */
  if (p == NULL)
    {
      /* remote host closed connection */
      es->state = ES_CLOSING;
      if(es->p == NULL)
      {
         /* we're done sending, close connection */
         tcp_echoserver_connection_close(tpcb, es);
      }
      else
      {
        /* we're not done yet */
        /* acknowledge received packet */


        /* send remaining data*/
        	tcp_sent(tpcb, tcp_echoserver_sent);
        	tcp_echoserver_send(tpcb, es);
      }
      ret_err = ERR_OK;
    }
    /* else : a non empty frame was received from client but for some reason err != ERR_OK */

  /* else : a non empty frame was received from client but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  }
  else if(es->state == ES_ACCEPTED)
  {
    /* first data chunk in p->payload */
  //  es->state = ES_RECEIVED;

   // tcp_command_recieved = 1;
    /* store reference to incoming pbuf (chain) */
 //   es->p = p;
	  flagtcp=1;
     cammand=p;
     AnswerToOrder((char*)cammand->payload,es);

    // es->p->payload="payload\r";






     if ((isUart6 == 1 && isUart6ITMode == 1) || (isUart2 == 1 && isUart2ITMode == 1))
     {

     }
     else
     {
        tcp_write(es->pcb,es->p->payload,26, 1);
        tcp_output(es->pcb);
     }





     flagtcp=0;


    /* initialize LwIP tcp_sent callback function */
  //  tcp_sent(tpcb, tcp_echoserver_sent);

     tcp_recved(tpcb, p->tot_len);
     es->p = NULL;
         pbuf_free(p);

  //  ans = AnswerToOrder(es);

    /* send back the received data (echo) */
 //  tcp_echoserver_send(tpcb, es);
     //tcp_recved(tpcb, p->tot_len);
    ret_err = ERR_OK;
  }
  else if(es->state == ES_CLOSING)
  {
    /* odd case, remote side closing twice, trash data */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  else
  {
    /* unkown es->state, trash data  */
    tcp_recved(tpcb, p->tot_len);
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_err callback function (called
  *         when a fatal tcp_connection error occurs. 
  * @param  arg: pointer on argument parameter 
  * @param  err: not used
  * @retval None
  */
void tcp_echoserver_error(void *arg, err_t err)
{
  struct tcp_echoserver_struct *es;
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(400);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(400);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(400);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);


  LWIP_UNUSED_ARG(err);

  es = (struct tcp_echoserver_struct *)arg;
  if (es != NULL)
  {
    mem_free(es);
  }


}

/**
  * @brief  This function implements the tcp_poll LwIP callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: pointer on the tcp_pcb for the current tcp connection
  * @retval err_t: error code
  */
err_t tcp_echoserver_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct tcp_echoserver_struct *es;
  HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
  //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
  //    HAL_Delay(1);
  //HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
  es = (struct tcp_echoserver_struct *)arg;
  if (es != NULL)
  {
    if (es->p != NULL)
    {

    }
    else
    {

    }
    ret_err = ERR_OK;
  }
  else
  {
    /* nothing to be done */
  //  tcp_abort(tpcb);
  //  ret_err = ERR_ABRT;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data) 
  * @param  None
  * @retval None
  */
err_t tcp_echoserver_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(len);
 // HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_SET);
  //  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
  //  HAL_Delay(200);
//    HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);
  //  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);

  es = (struct tcp_echoserver_struct *)arg;
  es->retries = 0;
  



  if(es->p != NULL)
  {
    /* still got pbufs to send */
    //tcp_echoserver_send(tpcb, es);

  }
  else
  {
    /* if no more data to send and client closed connection*/
    if(es->state == ES_CLOSING)
    {

      tcp_echoserver_connection_close(tpcb, es);
    }
  }
  flagtcp=0;
  return ERR_OK;
}


/**
  * @brief  This function is used to send data for tcp connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
void tcp_echoserver_send(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;
 
 // while ((wr_err == ERR_OK) &&
 //        (es->p != NULL) &&
 //        (es->p->len <= tcp_sndbuf(tpcb)))
  {
    
    /* get pointer on pbuf from es structure */
    ptr = es->p;


    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, 26, 1);

   if (wr_err == ERR_OK)
   {
     tcp_output(tpcb);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later / harder, defer to poll */
     es->p = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
}

/**
  * @brief  This functions closes the tcp connection
  * @param  tcp_pcb: pointer on the tcp connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
void tcp_echoserver_connection_close(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es)
{
  
  /* remove all callbacks */
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);
  
      	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
      	  HAL_Delay(100);

      	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
      	  HAL_Delay(100);

      	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
      	  HAL_Delay(100);

      	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
      	  HAL_Delay(100);

      	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
      	  HAL_Delay(100);

      	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
      	  HAL_Delay(100);

      	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
      	  HAL_Delay(100);

      	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
      	  HAL_Delay(100);

      	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
      	  HAL_Delay(100);

      	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
  /* delete es structure */
  if (es != NULL)
  {
    mem_free(es);
  }  
  
  /* close tcp connection */
  tcp_close(tpcb);
}

void AnswerToOrder(char* Command, struct tcp_echoserver_struct *es)
{
	//struct tcp_echoserver_struct *ans; ans = Command; //It is to ensure the length of ans
	isUart2 = 0;
	isUart6 = 0;
	int inputs[20];
	if (check(Command,"name"))
		es->p->payload = "SEM\r";
	else if (check(Command,"ver"))
		es->p->payload = "2017.1\r";
	else if (check(Command,"ip"))
		es->p->payload = "190.100.101.1\r";
	else if (check(Command,"port"))
		es->p->payload = "7\r";
	else if (check(Command,"sisel "))
	{
		int n = GetInputs(Command,inputs);
		//itoa(n,es->p->payload,10);
		if (n==1)
		{
		  ScanISel((uint8_t)inputs[0]);
	      es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}
	else if (check(Command,"strim "))
	{
		int n = GetInputs(Command,inputs);
		if (n==2)
		{
	      ScanTrim((uint8_t)inputs[0], (uint16_t)inputs[1]);
	      es->p->payload = "OK\r";
		}
		else if (n==3)
		{
	      ScanTrimij((uint8_t)inputs[0], (uint8_t)inputs[1], (uint16_t)inputs[2]);
	      es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:2,3\r";
		}
	}
	else if (check(Command,"zoomrot "))
	{
		int n = GetInputs(Command,inputs);
		if (n==3)
		{
		  ScanZoomRot((uint8_t)inputs[0], (uint8_t)inputs[1], (uint16_t)inputs[2]);
	      es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:3\r";
		}
	}
	else if (check(Command,"mux "))
	{
		int n = GetInputs(Command,inputs);
		//itoa(n,es->p->payload,10);
		if (n==1)
		{
		  DetMuxSel((uint8_t)inputs[0]);
	      es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}
	else if (check(Command,"dgain "))
	{
		int n = GetInputs(Command,inputs);
		//itoa(n,es->p->payload,10);
		if (n==3)
		{
		  DetAmpGain((uint8_t)inputs[0], (uint8_t)inputs[1], (uint8_t)inputs[2]);
	      es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:3\r";
		}
	}
	else if (check(Command,"dtrim "))
	{
		int n = GetInputs(Command,inputs);
		if (n==2)
		{
		  DetTrim((uint8_t)inputs[0], (uint16_t)inputs[1]);
	      es->p->payload = "OK\r";
		}
		else if (n==3)
		{
		  DetTrimij((uint8_t)inputs[0], (uint8_t)inputs[1], (uint16_t)inputs[2]);
	      es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:2,3\r";
		}
	}
	else if (check(Command,"led1 "))
	{
		int n = GetInputs(Command,inputs);
		if (n==1)
		{

			if ((uint8_t)inputs[0]==(uint8_t)0)
			{
				HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_RESET);
				es->p->payload = "OK\r";
			}
			else if ((uint8_t)inputs[0]==(uint8_t)1)
			{
				HAL_GPIO_WritePin(GPIOB, LD1_Pin, GPIO_PIN_SET);
				es->p->payload = "OK\r";
			}
			else
				es->p->payload = "err.Valid values:0,1\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}
	else if (check(Command,"led2 "))
	{
		int n = GetInputs(Command,inputs);
		if (n==1)
		{

			if ((uint8_t)inputs[0]==(uint8_t)0)
			{
				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
				es->p->payload = "OK\r";
			}
			else if ((uint8_t)inputs[0]==(uint8_t)1)
			{
				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
				es->p->payload = "OK\r";
			}
			else
				es->p->payload = "err.Valid values:0,1\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}
	else if (check(Command,"checkuart2"))
	{
		sendhello2(aRxBuffer);
		es->p->payload = aRxBuffer;
	}
	else if (check(Command,"checkuart6"))
	{
		sendhello6(aRxBuffer);
		es->p->payload = aRxBuffer;
	}
	else if (check(Command,"?."))
	{
		int ndata = GetChildCommand(Command, aTxBuffer);
		int nDeviceName = GetDeviceNameLength(aTxBuffer);
		ndata = nDeviceName + 6; //DeviceName.you?\r nDeviceName + 6
		char PureYouCommand[ndata];
		for (int i=0; i<ndata-1; i++) PureYouCommand[i] = aTxBuffer[i];
		PureYouCommand[ndata-1] = '\r';
		int n = GetInputs(Command,inputs);
		int isFound = 0;
		if (n==4)
		{
			if (inputs[0]==1 && isFound == 0)
			{
				FreeReceiveBuf2();
				if(HAL_UART_Transmit(&huart2, (uint8_t*)PureYouCommand, ndata, 500)== HAL_OK)
				{
					int isOK=1;
					int cnt=0;
					uint8_t receivedchar=' ';
					while ((receivedchar != '\r' && isOK==1) && cnt<=26)
					{
						if(HAL_UART_Receive(&huart2, &receivedchar, 1,1000)!= HAL_OK) {isOK=0;break;}
						aRxBuffer[cnt] = receivedchar;
						cnt++;
					}

					if(isOK==1 && IsSame(aRxBuffer,PureYouCommand, nDeviceName) == 1)
					{
						es->p->payload = "u2\r";
						isFound=1;
					}
				}
			}

			if (inputs[1]==1 && isFound == 0)
			{
				FreeReceiveBuf6();
				if(HAL_UART_Transmit(&huart6, (uint8_t*)PureYouCommand, ndata , 500)== HAL_OK)
				{
					int isOK=1;
					int cnt=0;
					uint8_t receivedchar=' ';
					while ((receivedchar != '\r' && isOK==1) && cnt<=26)
					{
						if(HAL_UART_Receive(&huart6, &receivedchar, 1,1000)!= HAL_OK) {isOK=0;break;}
						aRxBuffer[cnt] = receivedchar;
						cnt++;
					}

					if(isOK==1 && IsSame(aRxBuffer,PureYouCommand, nDeviceName) == 1)
					{
						es->p->payload = "u6\r";
						isFound=1;
					}
				}
			}

			if (inputs[2]==1 && isFound == 0)
			{
				isFound=0; //CAN1
			}

			if (inputs[3]==1 && isFound == 0)
			{
				isFound=0; //CAN1
			}

			if (isFound == 0) es->p->payload = "null\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:4\r";
		}
	}
	else if (check(Command,"u2."))
	{
		isUart2 = 1;
		FreeReceiveBuf2();
		int ndata = GetChildCommand(Command, aTxBuffer);
		if (isUart2ITMode == 1)
		{
			if(HAL_UART_Transmit_IT(&huart2, (uint8_t*)aTxBuffer, ndata) != HAL_OK)
			    es->p->payload = "err.sending error\r";
			if(HAL_UART_Receive_IT(&huart2, (uint8_t*)uRxBuffer, 2) != HAL_OK)
									      es->p->payload = "err.receive error\r";
		}
		else
		{
			if(HAL_UART_Transmit(&huart2, (uint8_t*)aTxBuffer, ndata, 2000)== HAL_OK)
			{
				int isOK=1;
				int cnt=0;
				uint8_t receivedchar=' ';
				while ((receivedchar != '\r' && isOK==1) && cnt<=26)
				{
					if(HAL_UART_Receive(&huart2, &receivedchar, 1,2000)!= HAL_OK) {isOK=0;break;}
					aRxBuffer[cnt] = receivedchar;
					cnt++;
				}

				//itoa(cnt,es->p->payload,10)es
				if(isOK==1)
					es->p->payload = aRxBuffer;
				else
					es->p->payload = "err.u2 time out 2s\r";
			}
			else
				es->p->payload = "err.sending time out\r";
		}
	}
	else if (check(Command,"u6."))
	{
		isUart6 = 1;
		int isSS = 0;
		if (check(Command,"u6.l.ss"))
        {
			isSS=1;//ss=single shot
			int n = GetInputs(Command,inputs);
			row=0;
			column=0;
		 	//*(uint32_t *) Dac_x = 0;  //Hystersis
		  //	*(uint32_t *) Dac_y = 0;
		 //   HAL_Delay(200);
			*(uint32_t *) Dac_x = 2047;
			*(uint32_t *) Dac_y = 2047;
        }


		//FreeReceiveBuf6();
		int ndata = GetChildCommand(Command, aTxBuffer);
		if (isUart6ITMode == 1)
		{
			if(HAL_UART_Transmit_IT(&huart6, (uint8_t*)aTxBuffer, ndata) != HAL_OK)
			      es->p->payload = "err.sending error\r";
			if(HAL_UART_Receive_IT(&huart6, (uint8_t*)uRxBuffer, 2) != HAL_OK)
						      es->p->payload = "err.receive error\r";
			//flag=1;
		}
		else
		{
			int isOK=1;
			if(HAL_UART_Transmit(&huart6, (uint8_t*)aTxBuffer, ndata, 2000)== HAL_OK)
			{
				int cnt=0;
				uint8_t receivedchar=' ';
				while ((receivedchar != '\r' && isOK==1) && cnt<=26)
				{
					if(HAL_UART_Receive(&huart6, &receivedchar, 1,2000)!= HAL_OK) {isOK=0;break;}
					aRxBuffer[cnt] = receivedchar;
					cnt++;
				}

				//itoa(cnt,es->p->payload,10);
				if(isOK==1)
					es->p->payload = aRxBuffer;
				else
					es->p->payload = "err.u6 time out 2s\r";
			}
			else
				es->p->payload = "err.sending time out\r";
		}

		if (isSS == 1) //ss=single shot
		{
			if (isSS==1) HAL_Delay(2);
			isSingleShot=1;
			SSDotNum = inputs[1] * inputs[1];
			SSiDot = 0;
			HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
		}
	}
	else if (check(Command,"c1."))
	{
		es->p->payload = "err.CAN1 is not programed.\r";
	}
	else if (check(Command,"c2."))
	{
		es->p->payload = "err.CAN2 is not programed.\r";
	}
	else if (check(Command,"dacxrange "))
	{
		int n = GetInputs(Command,inputs);
		if (n==2)
		{
			dacrange(xVoltageBuffer,(uint16_t)inputs[0],(uint16_t)inputs[1],512);//1.mean val and 2.amp
			es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:2\r";
		}
	}
	else if (check(Command,"dacyrange "))
	{
		int n = GetInputs(Command,inputs);
		if (n==2)
		{
			dacrange(yVoltageBuffer,(uint16_t)inputs[0],(uint16_t)inputs[1],512);//1.mean val and 2.amp
			es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:2\r";
		}
	}
	else if (check(Command,"dactimer "))
	{
		int n = GetInputs(Command,inputs);
		if (n==1)
		{

			if ((uint8_t)inputs[0]==(uint8_t)0)
			{
				isDacTimerOn = 0;
				//HAL_TIM_Base_Stop_IT(&htim2);
				HAL_TIM_Base_Stop(&htim4);
				//HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
				__HAL_TIM_DISABLE(&htim4);
				es->p->payload = "OK\r";
			}
			else if ((uint8_t)inputs[0]==(uint8_t)1)
			{
				isDacTimerOn = 1;
				//HAL_TIM_Base_Start_IT(&htim2);
				HAL_TIM_Base_Start(&htim4);
				//HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
				__HAL_TIM_ENABLE(&htim4);
				es->p->payload = "OK\r";
			}
			else
				es->p->payload = "err.Valid values:0,1\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}
	else if (check(Command,"dacper "))
	{
		int n = GetInputs(Command,inputs);
		if (n>=1)
		{
			if (inputs[0] > 0)
			{
				//HAL_TIM_Base_Stop(&htim4);
				//htim4.Init.Period = inputs[0];
				//htim4.Instance->CNT=60000;
				if(isDacTimerOn == 1)
				__HAL_TIM_DISABLE(&htim4);
				htim4.Instance->ARR=inputs[0];
				htim4.Instance->CCR4=inputs[1];
				if(isDacTimerOn == 1)
				__HAL_TIM_ENABLE(&htim4);
				//HAL_TIM_Base_Init(&htim4);
				//HAL_TIM_Base_Start(&htim4);
				es->p->payload = "OK\r";
			}
			else
				es->p->payload = "err.Valid values>0\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}

	else if (check(Command,"adctime "))
		{
			int n = GetInputs(Command,inputs);
			if (n==2)
			{
				if ((inputs[0] >= 0) && (inputs[0] <= 5))
				{
					hadc1.Instance->SMPR2 &= ~ADC_SMPR2(ADC_SMPR2_SMP0, ADC_CHANNEL_0);
					/* Set the new sample time */
					hadc1.Instance->SMPR2 |= ADC_SMPR2(inputs[0], ADC_CHANNEL_0);
					es->p->payload = "OK\r";

					/* Set ADC parameters */
					  /* Set the ADC clock prescaler */
					if ((inputs[1] >= 0) && (inputs[1] <= 3))
					{
					//	hadc1.CCR &= ~(ADC_CCR_ADCPRE);
					//	hadc1.CCR |=  ((uint32_t)input[1]) << (16U);
					}
				}
				else
					es->p->payload = "err.Valid values>0\r";
			}
			else
			{
				es->p->payload = "err.number of inputs:1\r";
			}
		}
	else if (check(Command,"dacxbuf "))
	{
		int n = GetInputs(Command,inputs);
		if (n==1)
		{
			if ((uint8_t)inputs[0]==(uint8_t)0)
			{
				DAC_ChannelConfTypeDef sConfig;
				sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
				sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
				HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
				es->p->payload = "OK\r";
			}
			else if ((uint8_t)inputs[0]==(uint8_t)1)
			{
				DAC_ChannelConfTypeDef sConfig;
				sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
				sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
				HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
				es->p->payload = "OK\r";
			}
			else
				es->p->payload = "err.Valid values:0,1\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}
	else if (check(Command,"dacybuf "))
	{
		int n = GetInputs(Command,inputs);
		if (n==1)
		{
			if ((uint8_t)inputs[0]==(uint8_t)0)
			{
				DAC_ChannelConfTypeDef sConfig;
				sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
				sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
				HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2);
				es->p->payload = "OK\r";
			}
			else if ((uint8_t)inputs[0]==(uint8_t)1)
			{
				DAC_ChannelConfTypeDef sConfig;
				sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
				sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
				HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2);
				es->p->payload = "OK\r";
			}
			else
				es->p->payload = "err.Valid values:0,1\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}
	else if (check(Command,"setsignal "))
	{
		int n = GetInputs(Command,inputs);
		if (n==4)
		{
			if (setsignal((uint8_t)inputs[0],(uint8_t)inputs[1],(uint8_t)inputs[2],(uint8_t)inputs[3])==1)
			{
				es->p->payload = "OK\r";
			}
			else
			{
				es->p->payload = "err.invalid values\r";
			}
		}
		else
		{
			es->p->payload = "err.number of inputs:4\r";
		}
	}
	else if (check(Command,"rotate "))
	{
		int n = GetInputs(Command,inputs);
		if (n==2)
		{
		  rotate((uint16_t)inputs[0], (uint16_t)inputs[1]);
	      es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:2\r";
		}
	}
	else if (check(Command,"zoom "))
	{
		int n = GetInputs(Command,inputs);
		if (n==2)
		{
		  zoom((uint16_t)inputs[0], (uint16_t)inputs[1]);
	      es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:2\r";
		}
	}
	else if (check(Command,"u2itmode "))
	{
		int n = GetInputs(Command,inputs);
		if (n==1)
		{
			if ((uint8_t)inputs[0] == (uint8_t)1)
			{
				isUart2ITMode=1;
				es->p->payload = "OK\r";
			}
			else if ((uint8_t)inputs[0] == (uint8_t)0)
			{
				isUart2ITMode=0;
				es->p->payload = "OK\r";
			}
			else
				es->p->payload = "err.Valid values:0,1\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}
	else if (check(Command,"u6itmode "))
	{
		int n = GetInputs(Command,inputs);
		if (n==1)
		{
			if ((uint8_t)inputs[0] == (uint8_t)1)
			{
				isUart6ITMode=1;
				es->p->payload = "OK\r";
			}
			else if ((uint8_t)inputs[0] == (uint8_t)0)
			{
				isUart6ITMode=0;
				es->p->payload = "OK\r";
			}
			else
				es->p->payload = "err.Valid values:0,1\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}
	else if (check(Command,"acquire "))
	{
		int n = GetInputs(Command,inputs);
		if (n==1)
		{
			if ((uint8_t)inputs[0]>(uint8_t)0)
			{
				isDacTimerOn = 0;
				HAL_TIM_Base_Stop_IT(&htim4);
				HAL_Delay(200);

				row=0;
				column=0;
				isAcquire=1;
				AcquireCnt=0;
				AcquireNumber=inputs[0];
				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);

				HAL_Delay(200);
				isDacTimerOn = 1;
				HAL_TIM_Base_Start_IT(&htim4);


				es->p->payload = "OK\r";
			}
			else
				es->p->payload = "err.Valid values:>0\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:1\r";
		}
	}
	else if (check(Command,"window "))
	{
		int n = GetInputs(Command,inputs);
		if (n==4)
		{
			__HAL_TIM_DISABLE(&htim4);
			HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
			__HAL_TIM_DISABLE(&htim4);
		//	windowset((uint16_t)inputs[0],(uint16_t)inputs[1],(uint16_t)inputs[2],(uint16_t)inputs[3]);
			wix = (uint16_t)inputs[0];
		    wiy = (uint16_t)inputs[1];
			wnx = (uint16_t)inputs[2];
			wny = (uint16_t)inputs[3];
            windowchanged=1;

			es->p->payload = "OK\r";
		}
		else
		{
			es->p->payload = "err.number of inputs:2\r";
		}
	}
	else if (check(Command,"winsize "))
		{
			int n = GetInputs(Command,inputs);
			if (n==1)
			{
				__HAL_TIM_DISABLE(&htim4);
				HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
				__HAL_TIM_DISABLE(&htim4);

				windowsize = (uint16_t)inputs[0];


	            dacrange(xVoltageBuffer,(uint16_t)2047, (uint16_t)2047,512*windowsize);
	            dacrange(yVoltageBuffer,(uint16_t)2047, (uint16_t)2047,512*windowsize);
	        //    windowchanged=1;
				es->p->payload = "OK\r";
			}
			else
			{
				es->p->payload = "err.number of inputs:2\r";
			}
		}
	else if (check(Command,"multiply "))
			{
				int n = GetInputs(Command,inputs);
				if (n==1)
				{
					__HAL_TIM_DISABLE(&htim4);
					HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
					__HAL_TIM_DISABLE(&htim4);
				//	windowset((uint16_t)inputs[0],(uint16_t)inputs[1],(uint16_t)inputs[2],(uint16_t)inputs[3]);
					multiply =  (uint16_t)inputs[0];
					//htim2.Instance->ARR = 0;//multiply-1;
					multiply_count = 1;
					//wnx = (uint16_t)inputs[2];
					//wny = (uint16_t)inputs[3];
		            windowchanged=2;
		           // dacrange(xVoltageBuffer,(uint16_t)2047, (uint16_t)2047,512);
		           // dacrange(yVoltageBuffer,(uint16_t)2047, (uint16_t)2047,512);
					es->p->payload = "OK\r";
				}
				else
				{
					es->p->payload = "err.number of inputs:2\r";
				}
			}
	else
		es->p->payload = "err.undefined command\r";
	//return;
}

int check(char* order,char* word)
{
	int is = 0;
	if (strncmp(order,word,strlen(word)) == 0) is=1;
	return is;
}

int GetInputs(char* Command, int* inputs)
{
	char buffer [20];
	int cnt;

	char str[50];
	strcpy(str,Command);
	char* pch;
	pch = strtok (str," ,\r");
	int num=0;
	cnt=0;
	while (pch != NULL)
	{
	  sprintf(buffer, "%s",pch);
          pch = strtok (NULL, " ,\r");
          if (num!=0)
          {
    	    inputs[cnt] = atoi(buffer);
    	    cnt++;
          }
          num=1;
	}
	return cnt-1;
}

int GetChildCommand(char* Command, char* ChildCommand)
{
	int istart = 0;
	for (int i=0; i<26; i++)
	{
		if (Command[i] == '.')
		{
			istart = i+1;
			break;
		}
	}

	int n = 0;
	for (int i=istart; i<26; i++)
	{
		ChildCommand[n]=Command[i];
		n++;
		if (Command[i] == '\r')
		{
			break;
		}
	}

	return n;
}

int GetDeviceNameLength(char* Command)
{
	int n = -1;
	for (int i=0; i<26; i++)
	{
		n++;
		if (Command[i] == '.')
		{
			break;
		}
	}
	return n;
}

int IsSame(char* s1,char* s2, int n)
{
	int nMax=26;
	if (n>nMax) n=nMax;
	int is=1;
	for (int i=0; i<n; i++)
	{
		if (s1[i] != s2[i])
		{
			is=0;
			break;
		}
	}
	return is;
}

//void GetChildCommand0(char* Command, char* ChildCommand)
//{
//	char* str = Command;
//	char* pch;
//	pch = strtok (str,":");
//	pch = strtok (NULL, ":");
//	sprintf(ChildCommand, "%s",pch);
//}

#endif /* LWIP_TCP */
