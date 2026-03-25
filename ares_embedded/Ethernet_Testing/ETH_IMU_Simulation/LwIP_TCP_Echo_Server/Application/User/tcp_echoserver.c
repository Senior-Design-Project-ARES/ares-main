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

// Header files
#include "tcp_echoserver.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include <string.h>	//for output string
#include <stdio.h>	//for output string
#include <math.h>	//for simulator (delete when not needed)

#if LWIP_TCP

//structure definitions
static struct tcp_pcb *tcp_echoserver_pcb;

/* Change this structure to match the needed output */
typedef struct {
	float ax, ay, az;	//acceleration
	float gx, gy, gz;	//angular acceleration
	float roll, pitch, yaw;	//orientation
} imu_packet;

/* ECHO protocol states */
enum tcp_echoserver_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

/* structure for maintaining connection infos to be passed as argument 
   to LwIP callbacks*/
struct tcp_echoserver_struct
{
  u8_t state;             /* current connection state */
  u8_t retries;
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
  float t;	//simulation timer
  char rx_buff[128];	//store incoming data
  int rx_len;	//length of incoming data
};


static err_t tcp_echoserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_echoserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_echoserver_error(void *arg, err_t err);
static err_t tcp_echoserver_poll(void *arg, struct tcp_pcb *tpcb);
static void tcp_echoserver_connection_close(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es);
static void generate_virtual_imu(imu_packet *imu, float t);


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
    err = tcp_bind(tcp_echoserver_pcb, IP_ADDR_ANY, 7);
    
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
static err_t tcp_echoserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  /* set priority for the newly accepted tcp connection newpcb */
  tcp_setprio(newpcb, TCP_PRIO_MIN);

  /* allocate structure es to maintain tcp connection information */
  es = (struct tcp_echoserver_struct *)mem_malloc(sizeof(struct tcp_echoserver_struct));
  if (es != NULL)
  {
    es->state = ES_ACCEPTED;
    es->pcb = newpcb;
    es->retries = 0;
    es->t = 0;
    es->rx_len = 0;
    
    /* pass newly allocated es structure as argument to newpcb */
    tcp_arg(newpcb, es);
    
    /* initialize lwip tcp_recv callback function for newpcb  */ 
    tcp_recv(newpcb, tcp_echoserver_recv);
    
    /* initialize lwip tcp_err callback function for newpcb  */
    tcp_err(newpcb, tcp_echoserver_error);
    
    /* initialize lwip tcp_poll callback function for newpcb */
    tcp_poll(newpcb, tcp_echoserver_poll, 1);
    
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
static err_t tcp_echoserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_echoserver_struct *es;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct tcp_echoserver_struct *)arg;
  
  /* if we receive an empty tcp frame from client => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    ret_err = ERR_OK;
  }   
  /* else : a non empty frame was received from client but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      pbuf_free(p);
    }
    ret_err = err;
  }
  else
  {
	  if (p->len > 0) //saftey check
	  {
		  int len = p->len;
		  //downsize the incoming data if too big
		  if (len > sizeof(es->rx_buff) -1)
			  len = sizeof(es->rx_buff) -1;

		  memcpy(es->rx_buff, p->payload, len);	//copy data
		  es->rx_buff[len] = '\0';	//add null terminator
		  es->rx_len = len;	//store the data length
	  }

	  // Acknowledge reception
	  tcp_recved(tpcb, p->tot_len);

	  // Free the packet
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
static void tcp_echoserver_error(void *arg, err_t err)
{
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(err);

  es = (struct tcp_echoserver_struct *)arg;
  if (es != NULL)
  {
    /*  free es structure */
    mem_free(es);
  }
}

/**
  * @brief  This function implements the tcp_poll LwIP callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: pointer on the tcp_pcb for the current tcp connection
  * @retval err_t: error code
  */
static err_t tcp_echoserver_poll(void *arg, struct tcp_pcb *tpcb)
{
	//variables
  struct tcp_echoserver_struct *es;
  es = (struct tcp_echoserver_struct *)arg;

  //check for null
  if (es == NULL)
      {
          tcp_abort(tpcb);
          return ERR_ABRT;
      }

  // Handle connection closing
  if (es->state == ES_CLOSING)
  {
	  tcp_echoserver_connection_close(tpcb, es);
	  return ERR_OK;
  }

  // --- Handle incoming command (if any) ---
  if (es->rx_len > 0)
  {
	  if (strncmp(es->rx_buff, "RESET", 5) == 0)
	  {
		  es->t = 0;
	  }

	  // clear command after processing
	  es->rx_len = 0;
  }

  // --- Generate IMU telemetry ---
  imu_packet imu;
  generate_virtual_imu(&imu, es->t);
  es->t += 0.05f;

  char msg[128];
  int len = sprintf(msg,
	  "ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f roll=%.2f pitch=%.2f yaw=%.2f\n",
	  imu.ax, imu.ay, imu.az,
	  imu.gx, imu.gy, imu.gz,
	  imu.roll, imu.pitch, imu.yaw);
  
  // --- Send telemetry ---
  if (len <= tcp_sndbuf(tpcb))
  {
	  tcp_write(tpcb, msg, len, TCP_WRITE_FLAG_COPY);
	  tcp_output(tpcb);
  }

  return ERR_OK;
}


/* Simulates IMU Data. Delete when not needed*/
static void generate_virtual_imu(imu_packet *imu, float t)
{
    imu->ax = sinf(t);
    imu->ay = cosf(t);
    imu->az = 9.81f;

    imu->gx = 0.1f * sinf(t);
    imu->gy = 0.2f * cosf(t);
    imu->gz = 0.5f;

    imu->roll  = 10.0f * sinf(t);
    imu->pitch = 5.0f  * cosf(t);
    imu->yaw   = fmodf(t * 20.0f, 360.0f);
}


/**
  * @brief  This functions closes the tcp connection
  * @param  tcp_pcb: pointer on the tcp connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void tcp_echoserver_connection_close(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es)
{
  
  /* remove all callbacks */
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);
  
  /* delete es structure */
  if (es != NULL)
  {
    mem_free(es);
  }  
  
  /* close tcp connection */
  tcp_close(tpcb);
}

#endif /* LWIP_TCP */

