//
// Created by devsploit on 2020/08/17.
//

#include "adc.h"
#include "logger.h"

static ADC_t adc = {0};

static inline void ADC_SAI_Interrupt_(ADC_state_flag_rec_e caller);

int ADC_init (I2C_HandleTypeDef *controlInterface, SAI_HandleTypeDef *audioInterface)
{

  adc.control = controlInterface;
  adc.audioPort = audioInterface;
  ADC_setState(ADC_SETUP);

  adc.wav.fname         = "REC";
  adc.wav.sampleRate    = 8000; // sps
  adc.wav.nChannels     = 2;
  adc.wav.bitsPerSample = 24;
  adc.wav.blockSize     = 4U; // bits

  WAVE_createFile(&adc.wav);


  ADC_reset();

  ADC_writeRegister(ADC_REG_PLL_CONTROL,
                    ADC_PLL_MUTE_ON |
                    ADC_CLK_S_MCLK |
                    ADC_MCS_768);

  ADC_writeRegister(ADC_REG_BLOCK_POWER_SAI,
                    ADC_LR_POL_LH
                    | ADC_BCLKEDGE_FALLING
                    | ADC_LDO_EN_ON
                    | ADC_VREG_EN_ON
                    | ADC_ADC_EN4_ON
                    | ADC_ADC_EN3_ON
                    | ADC_ADC_EN2_ON
                    | ADC_ADC_EN1_ON);

  ADC_writeRegister(ADC_REG_SAI_CTRL0,
                    ADC_SDATA_FMT_RJ_24
                    | ADC_SAI_TDM4
                    | ADC_FS_8_12);

  ADC_writeRegister(ADC_REG_SAI_CTRL1,
                    ADC_SDATA_SEL_2
                    | ADC_SLOT_WIDTH_32
                    | ADC_DATA_WIDTH_24
                    | ADC_LR_MODE_50
                    | ADC_SAI_MSB_MSB
                    | ADC_BCLKRATE_32
                    | ADC_SAI_MS_MASTER);

  ADC_writeRegister(ADC_REG_MISC_CONTROL,
                    ADC_SUM_MODE_2
                    | ADC_MMUTE_NONE
                    | ADC_DC_CAL_NO);

  if(ADC_powerUp() <= 0) {
    ERR("Failed to power up ADC Subsystems");
    return -2;
  }

  INFO("Waiting for PLL lock...");
  uint32_t tock = HAL_GetTick();
  while(!ADC_CMD_IS_PLL_LOCKED()){
    if((HAL_GetTick() - tock) > ADC_MAX_DELAY){
      ERR("PLL failed to lock (timeout)");
      return -1;
    }
  }

//  adc.dmaBuf = (uint8_t*)adc.buf;
  adc.bufDirty = (uint8_t*) adc.buf;
  INFO("DMA Buffer size: %u bytes", sizeof(adc.dmaBuf));
  INFO("Internal Buffer size: %u bytes", sizeof(adc.buf));


  ADC_setState(ADC_IDLE);
  return 1;
}

int ADC_yield ()
{

  if(ADC_is_err_set) {
    // Since `adc.state.flags` is a union and thus shares state with other enums its important to '&' with the bit field
    switch(adc.state.flags.err & ADC_FLAG_ERR_FIELD) {
      case ADC_ERR_N_REC: {
        WARN("Not recording with ADC running");
        ADC_setState(ADC_UNDEF);
        break;
      }

      case ADC_ERR_SAMPLE_MISSED: {
        WARN("Samples missed: %d", adc.samplesMissed);
        // TODO: tag samples missed
        // TODO: if more than % missed samples then reset device
        break;
      }

      default: WARN("Uncaught error 0x%02X", adc.state.flags);
    }
    ADC_clear_flag_err;
  }


  switch (adc.state.mode) {
    case ADC_REC: {
      if(ADC_is_interrupt_set) {
        INFO("Flushing buffer");
        const size_t dmaLen = sizeof(adc.dmaBuf) / 2;
        if (ADC_is_cplt_half) {
          WAVE_appendData(&adc.wav, adc.dmaBuf, dmaLen, 1);
        }
        else if(ADC_is_cplt_full) {
          WAVE_appendData(&adc.wav, adc.dmaBuf + dmaLen, dmaLen, 1);
        }
        ADC_clear_flag_cplt;
      }
      break;
    }

    case ADC_UNDEF: {
      ERR("ADC in undefined state!");
      // TODO: Better state recovery...
      for(;;);
    }
  }

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
  INFO("> I2C 0x%02X|0x%02X", registerAddr, rx);
  return rx;
}

// TODO: Create verified write that checks if wire was successful
int ADC_writeRegister(uint8_t registerAddr, uint8_t data)
{
  HAL_StatusTypeDef ret;
  INFO("< I2C 0x%02X|0x%02X", registerAddr, data);
  ret = HAL_I2C_Mem_Write(adc.control, ADC_I2C_ADDR,
                          registerAddr, ADC_REG_SIZE,
                          &data, 1,
                          ADC_MAX_DELAY);
  return (ret == HAL_OK);
}

void ADC_reset()
{
  HAL_GPIO_WritePin(ADC_nRST_GPIO_Port, ADC_nRST_Pin, GPIO_PIN_RESET);
  INFO("Resetting ADC");
  HAL_GPIO_WritePin(ADC_nRST_GPIO_Port, ADC_nRST_Pin, GPIO_PIN_SET);
}

int ADC_powerUp()
{
  INFO("Powering up ADC subsystems");
  return ADC_writeRegister(ADC_REG_M_POWER, ADC_PWUP);
}

int ADC_powerDown()
{
  WARN("Powering down ADC subsystems");
  return ADC_writeRegister(ADC_REG_M_POWER, ADC_PWUP_PWDWN);
}

ADC_state_major_e ADC_setState(ADC_state_major_e state)
{
  adc.state.mode = state;

  if(state == ADC_IDLE){
    INFO("Recorder IDLE");
    // TODO: Change to DMA pause and resume for better performance
    //stop recording
    HAL_SAI_DMAStop(adc.audioPort);
  }
  else if(state == ADC_REC) {
    // Start Recording
    INFO("Start recording");
    // Size is defined as frames and not bytes
    HAL_SAI_Receive_DMA(adc.audioPort, (uint8_t *) adc.dmaBuf, ADC_DMA_BUF_LEN);
  }

  switch (state) {
    case ADC_UNDEF: DBUG("Entering state: ADC_UNDEF"); break;
    case ADC_REC:   DBUG("Entering state: ADC_REC"); break;
    case ADC_SETUP: DBUG("Entering state: ADC_SETUP"); break;
    case ADC_IDLE:  DBUG("Entering state: ADC_IDLE"); break;
  }

  return adc.state.mode;
}

static inline void ADC_SAI_Interrupt_(ADC_state_flag_rec_e caller)
{
  // Interrupt already set... Samples missed
  if (ADC_is_interrupt_set) {
    adc.samplesMissed += ADC_DMA_BUF_LEN;
    adc.state.flags.err |= ADC_ERR_SAMPLE_MISSED;
  }
  else if (!ADC_is_recording) {
    adc.state.flags.err |= ADC_ERR_N_REC;
  }
  // Recording state
  else {
    HAL_GPIO_TogglePin(LED_STATUS_1_GPIO_Port, LED_STATUS_1_Pin);
    adc.state.flags.rec |= caller;
//    adc.dmaBuf += ADC_DMA_BUF_LEN;

//    const size_t len = adc.dmaBuf - adc.buf;
//    if(len == ADC_BUF_LEN) {
//      adc.dmaBuf = (uint8_t *) adc.buf;
//    }
//
////    adc.nSamples += ADC_DMA_BUF_LEN;
//      HAL_StatusTypeDef stat = HAL_SAI_Receive_DMA(adc.audioPort, adc.dmaBuf, ADC_DMA_BUF_LEN);
//    if( stat != HAL_OK){
//      ERR("HAL ERR");
//    }
  }
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  ADC_SAI_Interrupt_(ADC_CPLT_FULL);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  ADC_SAI_Interrupt_(ADC_CPLT_HALF);
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
  ERR("SAI Problem!");
}