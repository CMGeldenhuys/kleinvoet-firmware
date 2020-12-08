//
// Created by devsploit on 2020/08/17.
//

#include "adc.h"
#include "logger.h"

ADC_t adc = {0};

int ADC_init (I2C_HandleTypeDef *controlInterface, SAI_HandleTypeDef *audioInterface)
{

  adc.control = controlInterface;
  adc.audioPort = audioInterface;


  uint8_t tmp = ADC_readRegister(ADC_REG_M_POWER);
  INFO("M_POWER - 0x%02X", tmp);


  tmp = ADC_readRegister(0x10);
  INFO("0x10 - 0x%02X", tmp);

  tmp = ADC_readRegister(0x15);
  INFO("0x15 - 0x%02X", tmp);

  for(;;);



  return 1;
}

int ADC_yield ()
{
//  // TODO: Handle buffer overrun
//  // Half Complete
//  if (adc.rxPos == ADC_RX_LEN / 2) {
//    if (adc.storePtr != NULL) ERR("Persistence buffer overrun");
//    adc.storePtr = (uint32_t *) adc.rx;
//  }
//    // Full Complete
//  else if (adc.rxPos == ADC_RX_LEN) {
//    if (adc.storePtr != NULL) ERR("Persistence buffer overrun");
//    adc.storePtr = (uint32_t *) adc.rx[ADC_RX_LEN / 2];
//    adc.rxPos    = 0;
//  }
//
//  if (adc.storePtr != NULL) {
//    WAVE_appendData(&adc.wav, adc.storePtr, ADC_RX_LEN * ADC_NUM_CH * ADC_FRAME_SIZE/2, 1);
//    DBUG("Persisting ADC buffer");
//    adc.storePtr = NULL;
//  }
}

uint8_t ADC_readRegister(uint8_t registerAddr)
{
  uint8_t rx = 0;
  HAL_StatusTypeDef  ret = HAL_I2C_Mem_Read(adc.control, ADC_I2C_ADDR,
                   registerAddr, ADC_REG_SIZE,
                   &rx, 1, ADC_MAX_DELAY);
  if(ret != HAL_OK) ERR("I2C error");
  // if failed then return zeros
  // todo: better error handling
  return rx;
}

int ADC_writeRegister(uint8_t registerAddr, uint8_t data)
{
  HAL_StatusTypeDef ret;
  ret = HAL_I2C_Mem_Write(adc.control, ADC_I2C_ADDR,
                          registerAddr, ADC_REG_SIZE,
                          &data, 1,
                          ADC_MAX_DELAY);

  return (ret == HAL_OK);
}
