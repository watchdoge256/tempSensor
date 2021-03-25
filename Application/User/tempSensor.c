/*
 * tempSensor.c
 *
 *  Created on: 19.12.2020
 *      Author: amiar
 */
#include "tempSensor.h"

static void  Max31865_init(Max31865_t *max31865,SPI_HandleTypeDef *spi,GPIO_TypeDef  *cs_gpio,uint16_t cs_pin,uint8_t  numwires, uint8_t filterHz);
static bool  Max31865_readTempC(Max31865_t *max31865,float *readTemp);
static bool  Max31865_readTempF(Max31865_t *max31865,float *readTemp);
static float Max31865_Filter(float newInput, float lastOutput, float efectiveFactor);

/**
* @brief  TEMPSENSOR Process
* @param  argument: None
* @retval None
*/

#define CS_LO HAL_GPIO_WritePin(TEMPSENSOR_CS_GPIO_Port, TEMPSENSOR_CS_Pin, 0)
#define CS_HI HAL_GPIO_WritePin(TEMPSENSOR_CS_GPIO_Port, TEMPSENSOR_CS_Pin, 1)

void TEMPSENSOR_thread(void const * argument)
{
  static Max31865_t tempsensor;
  threadInfoArg_t * tempInfoArg = (threadInfoArg_t *)argument;
  SPI_HandleTypeDef *spi = (SPI_HandleTypeDef *)tempInfoArg->argument;

  Max31865_init(&tempsensor, spi, TEMPSENSOR_CS_GPIO_Port, TEMPSENSOR_CS_Pin, MAX31856_CONFIG_3WIRE, MAX31856_CONFIG_FILT50HZ);

  static float temperature;

  for( ;; )
  {
    Max31865_readTempC(&tempsensor, &temperature);
    osMessagePut(tempInfoArg->posMessageQIds[MQTT_queue_id], *(uint32_t *)&temperature, 1000);

    /* wait 250 ms */
    osDelay(1000);
  }
}


/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

/* SPI pins configuration ************************************************/
  /*
		SPI1_SCK  ----------------------> PA5
		SPI1_MISO ----------------------> PA6
		SPI1_MOSI ----------------------> PD7
		SPI1_CS   ----------------------> PA4
  */

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PD7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = TEMPSENSOR_CS_Pin|TEMPSENSOR_SCK_Pin|TEMPSENSOR_MISO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TEMPSENSOR_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(TEMPSENSOR_MOSI_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }

}


/**
* @brief SPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PD7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, TEMPSENSOR_CS_Pin|TEMPSENSOR_SCK_Pin|TEMPSENSOR_MISO_Pin);

    HAL_GPIO_DeInit(TEMPSENSOR_MOSI_GPIO_Port, TEMPSENSOR_MOSI_Pin);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }

}

//#########################################################################################################################
static void  Max31865_delay(uint32_t delay_ms)
{
  #if (_MAX31865_USE_FREERTOS == 1)
  osDelay(delay_ms);
  #else
  HAL_Delay(delay_ms);
  #endif
}
//#########################################################################################################################
static void Max31865_readRegisterN(Max31865_t *max31865,uint8_t addr, uint8_t *buffer, uint8_t n)
{
  uint8_t tmp = 0xFF;
  addr &= 0x7F;
  HAL_GPIO_WritePin(max31865->cs_gpio, max31865->cs_pin, GPIO_PIN_RESET);
  Max31865_delay(1);
  HAL_SPI_Transmit(max31865->spi,&addr, 1, 100);
  while (n--)
  {
    HAL_SPI_TransmitReceive(max31865->spi, &tmp, buffer, 1, 100);
    buffer++;
  }
  HAL_GPIO_WritePin(max31865->cs_gpio, max31865->cs_pin, GPIO_PIN_SET);
  Max31865_delay(1);
}
//#########################################################################################################################
static uint8_t Max31865_readRegister8(Max31865_t *max31865,uint8_t addr)
{
  uint8_t ret = 0;
  Max31865_readRegisterN(max31865, addr, &ret, 1);
  return ret;
}
//#########################################################################################################################
static uint16_t Max31865_readRegister16(Max31865_t *max31865,uint8_t addr)
{
  uint8_t buffer[2] = {0, 0};
  Max31865_readRegisterN(max31865, addr, buffer, 2);
  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |=  buffer[1];
  return ret;
}
//#########################################################################################################################
void Max31865_writeRegister8(Max31865_t *max31865,uint8_t addr, uint8_t data)
{
  HAL_GPIO_WritePin(max31865->cs_gpio, max31865->cs_pin, GPIO_PIN_RESET);
  Max31865_delay(1);
  addr |= 0x80;
  HAL_SPI_Transmit(max31865->spi,&addr, 1, 100);
  HAL_SPI_Transmit(max31865->spi,&data, 1, 100);
  HAL_GPIO_WritePin(max31865->cs_gpio, max31865->cs_pin, GPIO_PIN_SET);
  Max31865_delay(1);
}
//#########################################################################################################################
static uint8_t Max31865_readFault(Max31865_t *max31865)
{
  return Max31865_readRegister8(max31865, MAX31856_FAULTSTAT_REG);
}
//#########################################################################################################################
static void Max31865_clearFault(Max31865_t *max31865)
{
  uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31856_CONFIG_FAULTSTAT;
  Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}
//#########################################################################################################################
void Max31865_enableBias(Max31865_t *max31865, uint8_t enable)
{
  uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
  if (enable)
    t |= MAX31856_CONFIG_BIAS;
  else
    t &= ~MAX31856_CONFIG_BIAS;
  Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}
//#########################################################################################################################
static void Max31865_autoConvert(Max31865_t *max31865, uint8_t enable)
{
  uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
  if (enable)
    t |= MAX31856_CONFIG_MODEAUTO;
  else
    t &= ~MAX31856_CONFIG_MODEAUTO;
  Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}
//#########################################################################################################################
static void Max31865_setWires(Max31865_t *max31865, uint8_t numWires)
{
  uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
  if (numWires == 3)
    t |= MAX31856_CONFIG_3WIRE;
  else
    t &= ~MAX31856_CONFIG_3WIRE;
  Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}
//#########################################################################################################################
static void Max31865_setFilter(Max31865_t *max31865, uint8_t filterHz)
{
  uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
  if (filterHz == 50)
    t |= MAX31856_CONFIG_FILT50HZ;
  else
    t &= ~MAX31856_CONFIG_FILT50HZ;
  Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
}
//#########################################################################################################################
static uint16_t Max31865_readRTD (Max31865_t *max31865)
{
  Max31865_clearFault(max31865);
  Max31865_enableBias(max31865, 1);
  Max31865_delay(10);
  uint8_t t = Max31865_readRegister8(max31865, MAX31856_CONFIG_REG);
  t |= MAX31856_CONFIG_1SHOT;
  Max31865_writeRegister8(max31865, MAX31856_CONFIG_REG, t);
  Max31865_delay(65);
  uint16_t rtd = Max31865_readRegister16(max31865, MAX31856_RTDMSB_REG);
  rtd >>= 1;
  return rtd;
}
//#########################################################################################################################
//#########################################################################################################################
//#########################################################################################################################
static void  Max31865_init(Max31865_t *max31865,SPI_HandleTypeDef *spi,GPIO_TypeDef  *cs_gpio,uint16_t cs_pin,uint8_t  numwires, uint8_t filterHz)
{
  if(max31865->lock == 1)
    Max31865_delay(1);
  max31865->lock = 1;
  max31865->spi = spi;
  max31865->cs_gpio = cs_gpio;
  max31865->cs_pin = cs_pin;
  HAL_GPIO_WritePin(max31865->cs_gpio,max31865->cs_pin,GPIO_PIN_SET);
  Max31865_delay(100);
  Max31865_setWires(max31865, numwires);
  Max31865_enableBias(max31865, 0);
  Max31865_autoConvert(max31865, 0);
  Max31865_clearFault(max31865);
  Max31865_setFilter(max31865, filterHz);
}
//#########################################################################################################################
static bool Max31865_readTempC(Max31865_t *max31865,float *readTemp)
{
  if(max31865->lock == 1)
    Max31865_delay(1);
  max31865->lock = 1;
  bool isOk = false;
  float Z1, Z2, Z3, Z4, Rt, temp;
  Rt = Max31865_readRTD(max31865);
  Rt /= 32768;
  Rt *= _MAX31865_RREF;
  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / _MAX31865_RNOMINAL;
  Z4 = 2 * RTD_B;
  temp = Z2 + (Z3 * Rt);
  temp = (sqrtf(temp) + Z1) / Z4;

  if (temp >= 0)
  {
    *readTemp = temp;
    if(Max31865_readFault(max31865) == 0)
      isOk = true;
    max31865->lock = 0;
    return isOk;
  }
  Rt /= _MAX31865_RNOMINAL;
  Rt *= 100;
  float rpoly = Rt;
  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt;  // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt;  // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt;  // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt;  // ^5
  temp += 1.5243e-10 * rpoly;

  *readTemp = temp;
  if(Max31865_readFault(max31865) == 0)
    isOk = true;
  max31865->lock = 0;
  return isOk;
}
//#########################################################################################################################
static bool  Max31865_readTempF(Max31865_t *max31865,float *readTemp)
{
  bool isOk = Max31865_readTempC(max31865,readTemp);
  *readTemp = (*readTemp * 9.0f / 5.0f) + 32.0f;
  return isOk;
}
//#########################################################################################################################
static float Max31865_Filter(float newInput, float lastOutput, float efectiveFactor)
{
  return ((float)lastOutput*(1.0f-efectiveFactor)) + ((float)newInput*efectiveFactor) ;
}
//#########################################################################################################################
