//
// Created by devsploit on 2020/08/17.
//

#include "adc.h"
#include "logger.h"
#include <math.h>

static ADC_t adc = {0};

static inline void ADC_SAI_Interrupt_(ADC_state_flag_rec_e caller);
inline void ADC_32To24Blocks_(uint8_t *to, const uint32_t *from, size_t len, uint8_t lsb);
void ADC_persistBuf_(void * buf, size_t len);

int ADC_init (I2C_HandleTypeDef *controlInterface, SAI_HandleTypeDef *audioInterface)
{

  adc.control = controlInterface;
  adc.audioPort = audioInterface;
  ADC_setState(ADC_SETUP);

  adc.wav.fname         = "REC";
  adc.wav.sampleRate    = 48000; // sps
  adc.wav.nChannels     = 2;
  adc.wav.bitsPerSample = 24;
  adc.wav.blockSize     = 3U * adc.wav.nChannels; // bits

  WAVE_createFile(&adc.wav);


  ADC_reset();

  ADC_writeRegister(ADC_REG_PLL_CONTROL,
                    ADC_PLL_MUTE_ON |
                    ADC_CLK_S_MCLK |
                    ADC_MCS_256);

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
                    ADC_SDATA_FMT_I2S
                    // Note: the SAI interface does not leave the bits in the
                    // location they were received. It ALWAYS left justifies
                    // them to in the FIFO. Also note `Data size` of the SAI
                    // since the fifo only shifts in so many bits and then stops
                    | ADC_SAI_TDM4
                    | ADC_FS_32_48); // This register seems to subdivide FS?
                    // Does not actually correspond to FS...
                    // Leaving at default

  ADC_writeRegister(ADC_REG_SAI_CTRL1,
                    ADC_SDATA_SEL_2
                    | ADC_SLOT_WIDTH_32
                    | ADC_DATA_WIDTH_24
                    | ADC_LR_MODE_PULSE
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
//  adc.bufDirty = (uint8_t*) adc.buf;
  INFO("DMA Buffer size: %u bytes", sizeof(adc.dmaBuf));
//  INFO("Internal Buffer size: %u bytes", sizeof(adc.buf));
  HAL_SAI_DisableRxMuteMode(adc.audioPort);

  ADC_setState(ADC_IDLE);



#ifdef ADC_E2E_SYNTH
  {
    INFO("Creating synth");
    const size_t nChannels    = 2;
    const float freq[]        = {2e3f, 7e3f};
    const size_t fs           = 8000;
    const size_t   blockSize  = 512;
    const size_t   loop       = 100;
    const uint32_t amplitude  = 1U<<23U;
    const float  pi           = 3.1452f;
    const size_t len          = blockSize * nChannels * sizeof(int32_t);
    int32_t * data            = (int32_t *)malloc(len);

    for (size_t l = 0; l < loop; l++){
      for (size_t n = 0; n < blockSize; n++) {
        for (size_t c = 0; c < nChannels; c++){
          const float theta = 2.0f*pi*(float)(n+l*blockSize)*freq[c]/fs;
          const int32_t val = (int32_t) (amplitude * sinf(theta));
          *(data + (n * nChannels + c)) = val;
//          if ( c == 0)  *(data + (n * nChannels + c)) = __bswap32(0xFF00BEEFU);
//          else          *(data + (n * nChannels + c)) = __bswap32(0xFFDEAD00U);
        }

      }
      if(loop < 5 || l % 5 == 0) INFO("... done (%d)", l);
      ADC_persistBuf_((uint32_t*) data, len);
    }

    INFO("FIN -> %.2f secs", (float)(1.0f*loop*blockSize/fs));
    for(;;);
  }
#endif

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
        const size_t dmaLen = sizeof(adc.dmaBuf) / 2;
        uint8_t * dmaBuf = ADC_is_cplt_half
                ? (uint8_t*)adc.dmaBuf
                : (uint8_t*)adc.dmaBuf + dmaLen;

        DBUG("Flushing buffer (0x%08X -> %d)", dmaBuf, dmaLen);
        ADC_persistBuf_(dmaBuf, dmaLen);
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

void ADC_persistBuf_(void * buf, size_t len)
{
  ADC_32To24Blocks_(buf, buf, len, 0);
  WAVE_appendData(&adc.wav, buf, len*3/4, 1);
}

inline void ADC_32To24Blocks_(uint8_t *to, const uint32_t *from, size_t len, uint8_t lsb)
{
  const size_t del = 3;
  // Some awesome headache C pointer magic to introduce a one byte 'phase' shift in the from pointer;
  // Before : ... [FF][00][BE][EF][FF][00][DE][AD] ...
  //               ^---from
  //               |---to
  // After  : ... [FF][00][BE][EF][FF][00][DE][AD] ...
  //               ^   ^---from
  //               |---to
  // -------- idx = 0 --------
  // After  : ... [00][BE][EF][EF][FF][00][DE][AD] ...
  //                           ^   ^---from
  //                           |---to
  // -------- idx = 1 --------
  // After  : ... [00][BE][EF][00][DE][AD][DE][AD] ...
  //                                       ^       ^---from
  //                                       |---to
  // LSB = 0 : 0xFF000000 -> drops first byte
  // LSB = 1 : 0x000000FF -> drops last byte
  from = (const uint32_t *)((uint8_t*)(from) + lsb);
  for(size_t idx = 0;
      idx < len;
      idx+=3, from++, to += del) {
    // Use `memmove` instead of `memcpy` because is overlap safe
    memmove(to, from, del);
  }

//    for(size_t i = 0; i < len; i += 4,  from++, to += 3) {
//      const uint32_t val = *from & 0x00FFFFFFU; // take 24 bits
//      *to       = (val & 0x000000FFU) >>  0U;
//      *(to + 1) = (val & 0x0000FF00U) >>  8U;
//      *(to + 2) = (val & 0x00FF0000U) >> 16U;
//    }
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
    INFO("Recording started");
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
//    HAL_GPIO_TogglePin(LED_STATUS_1_GPIO_Port, LED_STATUS_1_Pin);
    adc.state.flags.rec |= caller;
    if(caller == ADC_CPLT_HALF)
      HAL_GPIO_WritePin(LED_STATUS_1_GPIO_Port, LED_STATUS_1_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(LED_STATUS_1_GPIO_Port, LED_STATUS_1_Pin, GPIO_PIN_RESET);
//    if(ADC_is_cplt_half)
//      HAL_GPIO_WritePin(LED_STATUS_1_GPIO_Port, LED_STATUS_1_Pin, GPIO_PIN_SET);
//    else if(ADC_is_cplt_full)
//      HAL_GPIO_WritePin(LED_STATUS_1_GPIO_Port, LED_STATUS_1_Pin, GPIO_PIN_RESET);
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

void ADC_incSample(void)
{
  DBUG("ADC FS pin toggled");

  // Only Inc while ADC is recording
  if(adc.state.mode == ADC_REC) {
    adc.nSamples++;
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