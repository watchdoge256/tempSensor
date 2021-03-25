/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/main.c 
  * @author  MCD Application Team
  * @brief   This sample code implements a http server application based on 
  *          Netconn API of LwIP stack and FreeRTOS. This application uses 
  *          STM32F7xx the ETH HAL API to transmit and receive data. 
  *          The communication is done with a web browser of a remote PC.
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
#include "main.h"
#include "ethernetif.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "app_ethernet.h"
#include "httpserver-netconn.h"
#include "tempSensor.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
typedef enum  {
  DHCP_thread_id = 0,
  MQTT_thread_id,
  TEMPSENSOR_thread_id,
}threadIds_e;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct netif gnetif; /* network interface structure */
SPI_HandleTypeDef gspi1;
GPIO_InitTypeDef gtempsensorCS;
osThreadId threadIds[3];
osMessageQDef_t MQTT_queue = {
    4,
    sizeof(float),
};
osMessageQId queueIds[3];
threadInfoArg_t temsensorInfoArg = {
    threadIds,
    queueIds,
    &gspi1,
};
threadInfoArg_t MQTTInfoArg = {
    threadIds,
    queueIds,
    &gnetif,
};

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void StartThread(void const * argument);
static void Netif_Config(void);
static void Error_Handler(void);
static void MPU_Config(void);
static void CPU_CACHE_Enable(void);
static void SPI1_Init(void);
static void GPIO_Init(void);


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* Configure the MPU attributes as Device memory for ETH DMA descriptors */
  MPU_Config();
  
  /* Configure the SPI for temperature sensor */
  SPI1_Init();
  GPIO_Init();

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();  
  
  /* Configure the system clock to 200 MHz */
  SystemClock_Config(); 
  
  /*configure LED1 and LED3 */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED3);

  /* Init thread */
#if defined(__GNUC__)
  osThreadDef(Start, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 5);
#else
  osThreadDef(Start, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
#endif
  
  osThreadCreate (osThread(Start), NULL);
  
  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */
  for( ;; );
}

/**
  * @brief  Start Thread 
  * @param  argument not used
  * @retval None
  */
static void StartThread(void const * argument)
{
  /* Create tcp_ip stack thread */
  tcpip_init(NULL, NULL);
  
  /* Initialize the LwIP stack */
  Netif_Config();
  
  /* Initialize webserver demo */
  http_server_netconn_init();
  
  /* Notify user about the network interface config */
  User_notification(&gnetif);
  
#ifdef USE_DHCP
  /* Start DHCPClient */
  osThreadDef(DHCP, DHCP_thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
  threadIds[DHCP_thread_id] = osThreadCreate (osThread(DHCP), &gnetif);
#endif

#ifdef USE_MQTT
  osThreadDef(MQTT, MQTT_thread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE * 2);
  threadIds[MQTT_thread_id] = osThreadCreate (osThread(MQTT), &MQTTInfoArg);
  queueIds[MQTT_thread_id] = osMessageCreate (&MQTT_queue, threadIds[MQTT_thread_id]);
#endif

#ifdef USE_TEMPSENSOR
  osThreadDef(TEMPSENSOR, TEMPSENSOR_thread, osPriorityHigh, 0, configMINIMAL_STACK_SIZE *2);
  threadIds[TEMPSENSOR_thread_id] = osThreadCreate (osThread(TEMPSENSOR), &temsensorInfoArg);
#endif

  for( ;; )
  {
    /* Delete the Init Thread */ 
    osThreadTerminate(NULL);
  }
}

/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
static void Netif_Config(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
 
#ifdef USE_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else
  IP_ADDR4(&ipaddr,IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
  IP_ADDR4(&netmask,NETMASK_ADDR0,NETMASK_ADDR1,NETMASK_ADDR2,NETMASK_ADDR3);
  IP_ADDR4(&gw,GW_ADDR0,GW_ADDR1,GW_ADDR2,GW_ADDR3);
#endif /* USE_DHCP */
  
  /* add the network interface */    
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
  
  /*  Registers the default network interface. */
  netif_set_default(&gnetif);
  
  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 400
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            PLL_R                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 7;  
  
  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the OverDrive */  
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; 
  
  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7);
  if(ret != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
}

/**
  * @brief  Configure the MPU attributes .
  * @param  None
  * @retval None
  */
static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;
  
  /* Disable the MPU */
  HAL_MPU_Disable();
  
  /* Configure the MPU as Normal Non Cacheable for Ethernet Buffers in the SRAM2 */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x2007C000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Configure the MPU as Device for Ethernet Descriptors in the SRAM2 */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x2007C000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Enable the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  Configure the MPU attributes .
  * @param  None
  * @retval None
  */
static void SPI1_Init(void)
{

  gspi1.Instance = SPI1;
  gspi1.Init.Mode = SPI_MODE_MASTER;
  gspi1.Init.Direction = SPI_DIRECTION_2LINES;
  gspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  gspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  gspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  gspi1.Init.NSS = SPI_NSS_SOFT;
  gspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  gspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  gspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  gspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  gspi1.Init.CRCPolynomial = 7;
  gspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  gspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&gspi1) != HAL_OK)
  {
    Error_Handler();
  }
	  /* USER CODE BEGIN SPI1_Init 2 */

	  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEMPSENSOR_CS_GPIO_Port, TEMPSENSOR_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TEMPSENSOR_CS_Pin */
  gtempsensorCS.Pin = TEMPSENSOR_CS_Pin;
  gtempsensorCS.Mode = GPIO_MODE_OUTPUT_PP;
  gtempsensorCS.Pull = GPIO_NOPULL;
  gtempsensorCS.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEMPSENSOR_CS_GPIO_Port, &gtempsensorCS);
  HAL_GPIO_WritePin(TEMPSENSOR_CS_GPIO_Port, TEMPSENSOR_CS_Pin, 0);
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
