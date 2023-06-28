/**
  ******************************************************************************
  * @file    LwIP/LwIP_UDP_Echo_Client/Src/udp_echoclient.c
  * @author  MCD Application Team
  * @brief   UDP echo client
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

u8_t   data[100];
uint32_t message_count = 0;
struct udp_pcb *upcb;

//#define DEST_IP_ADDR0  (u8_t)203
//#define DEST_IP_ADDR1  (u8_t)244
//#define DEST_IP_ADDR2  (u8_t)144
//#define DEST_IP_ADDR3  (u8_t)97
//#define UDP_SERVER_PORT	7
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Connect to UDP echo server
  * @param  None
  * @retval None
  */
void udp_echoclient_connect(void)
{
  ip_addr_t DestIPaddr;
  ip_addr_t myIPaddr;
  err_t err;
  
  /* Create a new UDP control block  */
  upcb = udp_new();
  
  if (upcb!=NULL)
  {
	  //dwm3000보드 아이피
	//ip_addr_t myIPaddr; //203.244.144.97
	//IP4_ADDR(&myIPaddr, 192, 168, 0, 104);
	//udp_bind(upcb, &myIPaddr, 8);

	  /*서버 아이피 */
    IP4_ADDR(&DestIPaddr, 192, 168, 1, 100);
    /* configure destination IP address and port */
    err= udp_connect(upcb, &DestIPaddr, 14147);

    if (err == ERR_OK)
    {
      /* Set a receive callback for the upcb */
      //udp_echoclient_send();
      udp_recv(upcb, udp_receive_callback, NULL);  
    }
  }
}
/*
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udp_echoclient_send(char* Values)
{
  struct pbuf *p;

  sprintf((char*)data, "%s", Values);

  /* allocate pbuf from pool*/
  p = pbuf_alloc(PBUF_TRANSPORT,strlen((char*)data), PBUF_POOL);

  if (p != NULL)
  {
    /* copy data to pbuf */
    pbuf_take(p, (char*)data, strlen((char*)data));

    /* send udp data */
    udp_send(upcb, p);

    /* free pbuf */
    pbuf_free(p);
  }
}

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{

  /*increment message count */
  message_count++;
  
  /* Free receive pbuf */
  pbuf_free(p);
}
