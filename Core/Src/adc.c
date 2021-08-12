/**
 *
 * @file adc.c
 * @author CM Geldenhuys
 * @date 17 Aug. 2020
 *
 * @headerfile adc.h "adc.h"
 *
 */

#include "adc.h"
#include "logger.h"
#include <math.h>
#include "timestamp.h"
#include "perf.h"

static ADC_t adc = {0};

// Stored in flash
const static uint8_t ADC_MISSED_SAMPLES_ZERO[ADC_DMA_BUF_LEN / 2 * 3 / 4] = {0};

static inline void ADC_SAI_Interrupt_ (ADC_state_flag_rec_e caller);

static inline void ADC_32To24Blocks_ (uint8_t *to, const uint32_t *from, size_t len, uint8_t lsb);

static void ADC_persistBuf_ (void *buf, size_t len, int * sync);

int ADC_init (I2C_HandleTypeDef *controlInterface, SAI_HandleTypeDef *audioInterface, TIM_HandleTypeDef *timInterface)
{

  adc.control   = controlInterface;
  adc.audioPort = audioInterface;
  adc.tim       = timInterface;
  ADC_setState(ADC_SETUP);

  adc.wav.fname         = ADC_FILENAME;
  adc.wav.sampleRate    = ADC_SAMPLING_RATE; // sps
  adc.wav.nChannels     = ADC_N_CHANNELS;
  adc.wav.bitsPerSample = 24;
  adc.wav.blockSize     = 3U * adc.wav.nChannels; // bits

  if (WAVE_createFile(&adc.wav) <= 0) return -1;


  ADC_reset();

  ADC_writeRegister(ADC_REG_PLL_CONTROL,
                    ADC_PLL_MUTE_ON |
                    ADC_CLK_S_MCLK |
#if   ADC_CRYSTAL_FREQ / ADC_SAMPLING_RATE == 128
                    ADC_MCS_128);
#elif ADC_CRYSTAL_FREQ / ADC_SAMPLING_RATE == 256
                    ADC_MCS_256);
#elif ADC_CRYSTAL_FREQ / ADC_SAMPLING_RATE == 384
                    ADC_MCS_384);
#elif ADC_CRYSTAL_FREQ / ADC_SAMPLING_RATE == 512
                    ADC_MCS_512);
#elif ADC_CRYSTAL_FREQ / ADC_SAMPLING_RATE == 768
                    ADC_MCS_768);
#else
#error "Unable to configure ADC sampling rate using current crystal"
#endif


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

#ifdef ADC_EN_HPF
  WARN("Enabling built-in HPF");
  ADC_writeRegister(ADC_REG_HPF_CAL,
                    ADC_DC_SUB_C4_OFF
                    | ADC_DC_SUB_C3_OFF
                    | ADC_DC_SUB_C2_OFF
                    | ADC_DC_SUB_C1_OFF
                    // Enable HPF
                    | ADC_DC_HPF_C4_ON
                    | ADC_DC_HPF_C3_ON
                    | ADC_DC_HPF_C2_ON
                    | ADC_DC_HPF_C1_ON);
#endif
  if (ADC_powerUp() <= 0) {
    ERR("Failed to power up ADC Subsystems");
    return -2;
  }

  INFO("Waiting for PLL lock...");
  uint32_t tock = HAL_GetTick();
  while (!ADC_CMD_IS_PLL_LOCKED()) {
    if ((HAL_GetTick() - tock) > ADC_MAX_DELAY) {
      ERR("PLL failed to lock (timeout)");
      return -1;
    }
  }

  INFO("Allocating DMA Buffer size: %u bytes", ADC_DMA_BUF_LEN);
  adc.dmaBuf = (uint8_t *) malloc(ADC_DMA_BUF_LEN);

  if (adc.dmaBuf == NULL) {
    ERR("Failed to allocate DMA buffer to heap!");
    return -2;
  }
  // Disable any mute mode as this can cause problems in the future
  // if implemented would require more OOP state machine
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

int ADC_yield (int * sync)
{

  // TODO: Track delta between DMA samples and TIM samples
  // If drift becomes to large one can warn that the MCU is running too slow or
  // some tasks are taking too long

  if (ADC_is_err_set) {
    // Since `adc.state.flags` is a union and thus shares state with other enums its important to '&' with the bit field
    switch (adc.state.flags.err & ADC_FLAG_ERR_FIELD) {
      case ADC_ERR_N_REC: {
        WARN("Not recording with ADC running");
        ADC_setState(ADC_UNDEF);
        break;
      }

      case ADC_ERR_SAMPLE_MISSED: {
        const float lossRate = adc.nFramesMissed * 100.0f / adc.nFrames;
        WARN("Frames missed: %d (%.2f%%)", adc.nFramesMissed, lossRate);
        DBUG("Persisting zeros for missed samples");
        WAVE_appendData(&adc.wav, ADC_MISSED_SAMPLES_ZERO, sizeof(ADC_MISSED_SAMPLES_ZERO), 0);
        // TODO: if more than % missed samples then reset device
        break;
      }

      default:
        WARN("Uncaught error 0x%02X", adc.state.flags);
    }
    ADC_clear_flag_err;
  }


  switch (adc.state.mode) {
    case ADC_REC: {
      if (ADC_is_interrupt_set) {
        const size_t dmaLen = ADC_DMA_BUF_LEN / 2;
        uint8_t *dmaBuf = ADC_is_cplt_half
                          ? (uint8_t *) adc.dmaBuf
                          : (uint8_t *) adc.dmaBuf + dmaLen;

        DBUG("Flushing buffer (0x%08X -> %d)", dmaBuf, dmaLen);
        ADC_persistBuf_(dmaBuf, dmaLen, sync);
        ADC_clear_flag_cplt;
      }
      break;
    }

    case ADC_UNDEF: {
      ERR("ADC in undefined state!");
      Error_Handler();
      break;
    }

    case ADC_IDLE:
      break;

    case ADC_SETUP: {
      WARN("ADC not ready!");
      break;
    }

    case ADC_STOP: {
      ERR("ADC STOPPED HALTING DEVICE");
      for(;;);
    }
  }

  return 1;
}

static void ADC_persistBuf_ (void *buf, size_t len, int * sync)
{
  ADC_32To24Blocks_(buf, buf, len, 0);
  int bw = WAVE_appendData(&adc.wav, buf, len * 3 / 4, *sync);
  if(bw > 0 && *sync > 0){
    INFO("Syncing WAVE changes to FAT FS");
    // Clear flag
    *sync = 0;
  }
}

static inline void ADC_32To24Blocks_ (uint8_t *to, const uint32_t *from, size_t len, uint8_t lsb)
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
  from = (const uint32_t *) ((uint8_t *) (from) + lsb);
  for (size_t idx = 0;
       idx < len;
       idx += 3, from++, to += del) {
    // Use `memmove` instead of `memcpy` because is overlap safe
    memmove(to, from, del);
  }
}

uint8_t ADC_readRegister (uint8_t registerAddr)
{
  uint8_t           rx  = 0;
  HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(adc.control, ADC_I2C_ADDR,
                                           registerAddr, ADC_REG_SIZE,
                                           &rx, 1, ADC_MAX_DELAY);
  if (ret != HAL_OK) ERR("I2C error");
  // if failed then return zeros
  // todo: better error handling
  INFO("> I2C 0x%02X|0x%02X", registerAddr, rx);
  return rx;
}

// TODO: Create verified write that checks if wire was successful
int ADC_writeRegister (uint8_t registerAddr, uint8_t data)
{
  HAL_StatusTypeDef ret;
  INFO("< I2C 0x%02X|0x%02X", registerAddr, data);
  ret = HAL_I2C_Mem_Write(adc.control, ADC_I2C_ADDR,
                          registerAddr, ADC_REG_SIZE,
                          &data, 1,
                          ADC_MAX_DELAY);
  // Allow time to latch
  HAL_Delay(10);
  return (ret == HAL_OK);
}

void ADC_reset ()
{
  HAL_GPIO_WritePin(ADC_nRST_GPIO_Port, ADC_nRST_Pin, GPIO_PIN_RESET);
  INFO("Resetting ADC");
  // Allow time to latch
  HAL_Delay(10);
  HAL_GPIO_WritePin(ADC_nRST_GPIO_Port, ADC_nRST_Pin, GPIO_PIN_SET);
}

int ADC_powerUp ()
{
  INFO("Powering up ADC subsystems");
  return ADC_writeRegister(ADC_REG_M_POWER, ADC_PWUP);
}

int ADC_powerDown ()
{
  WARN("Powering down ADC subsystems");
  return ADC_writeRegister(ADC_REG_M_POWER, ADC_PWUP_PWDWN);
}

int ADC_setState (ADC_state_major_e state)
{

  switch(state) {
    case ADC_IDLE: {
      LED_ORANGE_SET_LOW();
      INFO("Recorder IDLE");
      HAL_SAI_DMAPause(adc.audioPort);
      HAL_TIM_Base_Stop(adc.tim);
      break;
    }

    case ADC_STOP:{
      if (adc.state.mode != ADC_REC){
        WARN("ADC not recording, can not stop");
        return -1;
      }
      INFO("Recording stopping...");
      LED_ORANGE_SET_LOW();
      // Stop Peripherals
      HAL_SAI_DMAStop(adc.audioPort);
      HAL_TIM_Base_Stop(adc.tim);

      // Close WAVE File
      WAVE_close(&adc.wav);
      break;
    }

    case ADC_START:
    case ADC_REC: {
      if(adc.state.mode != ADC_IDLE) {
        ERR("ADC not ready!");
        return -1;
      }
      // Start Recording
      INFO("Recording started");
      LED_ORANGE_SET_HIGH();
      // Size is defined as frames and not bytes
      // This is due to the FIFO buffer used
      HAL_StatusTypeDef ret;
      ret = HAL_SAI_Receive_DMA(adc.audioPort, adc.dmaBuf, ADC_N_FRAMES);
      if (ret != HAL_OK) return -1;

      ret = HAL_TIM_Base_Start(adc.tim);
      if (ret != HAL_OK) return -2;
      break;
    }

    // TODO Implement
    case ADC_RESUME:
    case ADC_PAUSE: {
      ERR("Not implemented");
      return -3;
    }

    case ADC_SETUP:
      break;

    default: {
      WARN("Unknown state (0x%08X", state);
    }
  }
#ifdef LOG_LEVEL_DEBUG
  switch (state) {
    case ADC_UNDEF:   DBUG("Entering state: ADC_UNDEF");  break;
    case ADC_REC:     DBUG("Entering state: ADC_REC");    break;
    case ADC_SETUP:   DBUG("Entering state: ADC_SETUP");  break;
    case ADC_IDLE:    DBUG("Entering state: ADC_IDLE");   break;
    case ADC_STOP:    DBUG("Entering state: ADC_STOP");   break;
    case ADC_PAUSE:   DBUG("Entering state: ADC_PAUSE");  break;
    case ADC_RESUME:  DBUG("Entering state: ADC_RESUME"); break;
    case ADC_START:   DBUG("Entering state: ADC_START");  break;
    default:          DBUG("Entering unknown state");     break;
  }
#endif

  adc.state.mode = state;
  return 1;
}

static inline void ADC_SAI_Interrupt_ (ADC_state_flag_rec_e caller)
{
  // Interrupt already set... Samples missed
  if (ADC_is_interrupt_set) {
    TIME_meta("samples missed");
    adc.nFramesMissed++;
    adc.nFrames++;

    adc.state.flags.err |= ADC_ERR_SAMPLE_MISSED;
  }
  else if (!ADC_is_recording) {
    adc.state.flags.err |= ADC_ERR_N_REC;
  }
    // Recording state
  else {
    adc.nFrames++;
    adc.state.flags.rec |= caller;
  }
}

void HAL_SAI_RxCpltCallback (SAI_HandleTypeDef *hsai)
{
  ADC_SAI_Interrupt_(ADC_CPLT_FULL);
}

void HAL_SAI_RxHalfCpltCallback (SAI_HandleTypeDef *hsai)
{
  ADC_SAI_Interrupt_(ADC_CPLT_HALF);
}

void HAL_SAI_ErrorCallback (SAI_HandleTypeDef *hsai)
{
  ERR("SAI Problem! (0x%08x)", HAL_SAI_GetState(hsai));
}

inline void ADC_WAVE_writeHeader()
{
  WAVE_writeHeader(&adc.wav);
}

// TODO: Not the best place but works for now
int ADC_updateLocation(const int32_t ecef[3], uint32_t pAcc)
{
  static volatile uint32_t prevAcc = UINT32_MAX;

  if ( pAcc < prevAcc ){
    prevAcc = pAcc;
    const int32_t ecefX = ecef[0];
    const int32_t ecefY = ecef[1];
    const int32_t ecefZ = ecef[2];
    INFO("ECEF Update: %d,%d,%d", ecefX, ecefY, ecefZ);
    return WAVE_infoChunkPrintf(&adc.wav, WAVE_INFO_IDX_LOCATION, "%08X-%08X-%08X", ecefX, ecefY, ecefZ);
  }
  else{
    INFO("No update, current loc. kept");
    return 0;
  }

}

#ifdef DEBUG
int CMD_comment(int argc, char * argv[])
{
  if( argc > 0) {
    return WAVE_infoChunkPrintf(&adc.wav, WAVE_INFO_IDX_COMMENT, "%s", *argv);
  }
  return -1;
}
#endif