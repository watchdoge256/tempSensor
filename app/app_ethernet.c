/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/app_ethernet.c 
  * @author  MCD Application Team
  * @brief   Ethernet specefic module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
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
#include "lwip/opt.h"
#include "main.h"
#include "lwip/dhcp.h"
#include "app_ethernet.h"
#include "ethernetif.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include <math.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef USE_DHCP
#define MAX_DHCP_TRIES  4
__IO uint8_t DHCP_state = DHCP_OFF;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void mqtt_pub_request_cb(void *arg, err_t result);
/**
  * @brief  Notify the User about the network interface config status
  * @param  netif: the network interface
  * @retval None
  */
void User_notification(struct netif *netif) 
{
  if (netif_is_up(netif))
  {
#ifdef USE_DHCP
    /* Update DHCP state machine */
    DHCP_state = DHCP_START;
#else
    /* Turn On LED 1 to indicate ETH and LwIP init success*/
    BSP_LED_On(LED1);
#endif /* USE_DHCP */
  }
  else
  {  
#ifdef USE_DHCP
    /* Update DHCP state machine */
    DHCP_state = DHCP_LINK_DOWN;
#endif  /* USE_DHCP */
   /* Turn On LED 3 to indicate ETH and LwIP init error */
   BSP_LED_On(LED3);
  } 
}

#ifdef USE_DHCP
/**
* @brief  DHCP Process
* @param  argument: network interface
* @retval None
*/
void DHCP_thread(void const * argument)
{
  struct netif *netif = (struct netif *) argument;
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  struct dhcp *dhcp;
  
  for (;;)
  {
    switch (DHCP_state)
    {
    case DHCP_START:
      {
        ip_addr_set_zero_ip4(&netif->ip_addr);
        ip_addr_set_zero_ip4(&netif->netmask);
        ip_addr_set_zero_ip4(&netif->gw);       
        dhcp_start(netif);
        DHCP_state = DHCP_WAIT_ADDRESS;
      }
      break;
      
    case DHCP_WAIT_ADDRESS:
      {                
        if (dhcp_supplied_address(netif)) 
        {
          DHCP_state = DHCP_ADDRESS_ASSIGNED;	
          
          BSP_LED_Off(LED3);
          BSP_LED_On(LED1); 
        }
        else
        {
          dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);
    
          /* DHCP timeout */
          if (dhcp->tries > MAX_DHCP_TRIES)
          {
            DHCP_state = DHCP_TIMEOUT;
            
            /* Stop DHCP */
            dhcp_stop(netif);
            
            /* Static address used */
            IP_ADDR4(&ipaddr, IP_ADDR0 ,IP_ADDR1 , IP_ADDR2 , IP_ADDR3 );
            IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
            IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
            netif_set_addr(netif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));

            BSP_LED_Off(LED3);
            BSP_LED_On(LED1);
            
          }
          else
          {
            BSP_LED_On(LED3);
          }
        }
      }
      break;
  case DHCP_LINK_DOWN:
    {
      /* Stop DHCP */
      dhcp_stop(netif);
      DHCP_state = DHCP_OFF; 
    }
    break;
    default: break;
    }
    
    /* wait 250 ms */
    osDelay(250);
  }
}
#endif  /* USE_DHCP */

/* MQTT enabled only if DHCP and MQTT are in use */
#ifdef USE_MQTT

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {


 if(status == MQTT_CONNECT_ACCEPTED) {
   printf("mqtt_connection_cb: Successfully connected\n");

//   /* Setup callback for incoming publish requests */
//   mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);
//
//   /* Subscribe to a topic named "subtopic" with QoS level 1, call mqtt_sub_request_cb with result */
//   err = mqtt_subscribe(client, "subtopic", 1, mqtt_sub_request_cb, arg);

//   if(err != ERR_OK) {
//     printf("mqtt_subscribe return: %d\n", err);
//   }
// } else {
//   printf("mqtt_connection_cb: Disconnected, reason: %d\n", status);
 }
}


void MQTT_thread(void const * argument)
{
  err_t err;
  static mqtt_client_t client;
	struct mqtt_connect_client_info_t client_info = {
	  "imbryk",
	  NULL, NULL,
	  10,
	  NULL, NULL, 0, 0
	};
	static uint8_t minute;
	static float minuteval;
  const ip_addr_t test_mqtt_remote_ip = IPADDR4_INIT_BYTES(34,213,50,101);
  threadInfoArg_t *threadInfo = (threadInfoArg_t *)argument;
  osEvent osEv;
  char temperature[50];
  float temp;
  char *tmpSign;
  float tmpVal;

  int tmpInt1;                  // Get the integer (678).
  float tmpFrac;      // Get fraction (0.0123).
  int tmpInt2;  // Turn into integer (123).

  for(;;)
	{
	  if (DHCP_state == DHCP_ADDRESS_ASSIGNED){
		  if (mqtt_client_is_connected(&client)){
		    osEv = osMessageGet(threadInfo->posMessageQIds[MQTT_queue_id], 1);
		    if (osEv.status == osEventMessage){
		      temp = *(float*)&osEv.value.v;
		      tmpSign = (temp < 0) ? "-" : "";
		      tmpVal = (temp < 0) ? -temp : temp;
		      tmpInt1 = tmpVal;
		      tmpFrac = tmpVal - tmpInt1;
		      tmpInt2 = trunc(tmpFrac * 10000);
		      sprintf (temperature, "%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
		      err =  mqtt_publish(&client, "/devices/1s/temp", temperature, 7, 0, 0, mqtt_pub_request_cb, NULL);
		      if (minute == 60) {
		        minuteval = minuteval/60;
	          tmpSign = (minuteval < 0) ? "-" : "";
	          tmpVal = (minuteval < 0) ? -minuteval : minuteval;
	          tmpInt1 = tmpVal;
	          tmpFrac = tmpVal - tmpInt1;
	          tmpInt2 = trunc(tmpFrac * 10000);
	          sprintf (temperature, "%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
	          err =  mqtt_publish(&client, "/devices/1m/temp", temperature, 7, 0, 0, mqtt_pub_request_cb, NULL);
	          minute = 0;
		      }
		      else {
		        minute++;
		        minuteval += temp;
		      }
		    }
			}
		  else {
		    err = mqtt_client_connect(&client, &test_mqtt_remote_ip, 1883, mqtt_connection_cb, NULL, &client_info);
		  }
	  }
		osDelay(500);
	}
}

static void mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
    printf("Publish result: %d\n", result);
  }
}

#endif /* MQTT enabled only if DHCP and MQTT are in use */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
