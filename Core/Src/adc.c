//
// Created by devsploit on 2020/08/17.
//

#include "adc.h"
#include "logger.h"

static inline int32_t ADC_bswap32_24_ (uint8_t const frame[3]);

static inline void ADC_sample_ ();

ADC_t adc = {0};

int ADC_init (SPI_HandleTypeDef *interface)
{
  adc.spi   = interface;
  adc.state = ADC_IDLE; // Block interrupts

  // TODO: Get rid of magic numbers\
  // TODO: Add recorder ID
  adc.wav.fname         = "REC";
  adc.wav.sampleRate    = 4000; // sps
  adc.wav.nChannels     = 4;
  adc.wav.bitsPerSample = 24;
  adc.wav.blockSize     = 24; // bits

  WAVE_createFile(&adc.wav);

  if (ADC_CS_IS_ENABLE()) {
    WARN("ADC CS low (unknown state)");
    ADC_CS_DISABLE();
    HAL_Delay(ADC_WAIT_DELTA_);
  }

  ADC_sendCommand(ADC_CMD_OP_RESET, 0);
  DBUG("Reset: 0x%06X", ADC_sendCommand(ADC_CMD_OP_NULL, 0));

  while (!ADC_IS_READY());

  ADC_sendCommand(ADC_CMD_OP_RREG | ADC_ADR_ID, 0);
  DBUG("ID: 0x%06X", ADC_sendCommand(ADC_CMD_OP_NULL, 0));

  ADC_sendCommand(ADC_CMD_OP_WREG | ADC_ADR_CLOCK,
                  ADC_CLOCK_CH3_EN
                  | ADC_CLOCK_CH2_EN
                  | ADC_CLOCK_CH1_EN
                  | ADC_CLOCK_CH0_EN
                  | ADC_CLOCK_OSR_256
                  | ADC_CLOCK_PWR_HR);
  DBUG("WREG - CLK: 0x%06X", ADC_sendCommand(ADC_CMD_OP_NULL, 0));

  adc.state = ADC_READY | ADC_FIRST_READ;
  return 1;
}

int ADC_yield ()
{
  // TODO: Handle buffer overrun
  // Half Complete
  if (adc.rxPos == ADC_RX_LEN / 2) {
    if (adc.storePtr != NULL) ERR("Persistence buffer overrun");
    adc.storePtr = (uint32_t *) adc.rx;
  }
    // Full Complete
  else if (adc.rxPos == ADC_RX_LEN) {
    if (adc.storePtr != NULL) ERR("Persistence buffer overrun");
    adc.storePtr = (uint32_t *) adc.rx[ADC_RX_LEN / 2];
    adc.rxPos    = 0;
  }

  if (adc.storePtr != NULL) {
    WAVE_appendData(&adc.wav, adc.storePtr, ADC_RX_LEN * ADC_NUM_CH * ADC_FRAME_SIZE/2, 1);
    DBUG("Persisting ADC buffer");
    adc.storePtr = NULL;
  }
}

static inline int32_t ADC_bswap32_24_ (uint8_t const frame[3])
{
  return (frame[0] & 0x80U ? 0xFFFFU << 24U : 0x0000U << 24U) // sign extension
         | (unsigned) (frame[0] << 16u)
         | (unsigned) (frame[1] << 8u)
         | (unsigned) frame[2];
}

static inline void ADC_sample_ ()
{
  ADC_CS_ENABLE();
  HAL_SPI_Receive(adc.spi, (uint8_t *) adc.spiRx, (ADC_FRAME_NUM * ADC_FRAME_SIZE), ADC_TIMEOUT);
  ADC_CS_DISABLE();

  // Convert from Big Endian to Little Endian and persist
  for (size_t i = 0; i < ADC_NUM_CH; i++) {
    // TODO: Improve...
    adc.rx[adc.rxPos][i][2] = adc.spiRx[i + ADC_RESPONSE_LEN][0];
    adc.rx[adc.rxPos][i][1] = adc.spiRx[i + ADC_RESPONSE_LEN][1];
    adc.rx[adc.rxPos][i][0] = adc.spiRx[i + ADC_RESPONSE_LEN][2];
  }

  // Increment counters
  adc.rxPos++;
  adc.sampleCount++;

}

int ADC_sendCommand (uint16_t cmd, uint16_t opt)
{
  HAL_StatusTypeDef status;
  // Little Endian to Big Endian with zero padding
  uint8_t           tx[ADC_FRAME_NUM][ADC_FRAME_SIZE] = {{cmd >> 8u, cmd & 0xFFu}};
  // TODO: Fix hardcode
  if ((cmd & ADC_CMD_OP_WREG) == ADC_CMD_OP_WREG) {
    tx[1][0] = ((opt & 0x0000FF00U) >> 8U);
    tx[1][1] = ((opt & 0x000000FFU) >> 0U);
  }

  ADC_CS_ENABLE();
  status = HAL_SPI_TransmitReceive(adc.spi,
                                   (uint8_t *) tx,
                                   (uint8_t *) adc.spiRx,
                                   (ADC_FRAME_NUM * ADC_FRAME_SIZE),
                                   (ADC_TIMEOUT));
  ADC_CS_DISABLE();

  // Convert from Big Endian to Little Endian
  int ret = ADC_bswap32_24_(adc.spiRx[0]);

  return status == HAL_OK ? ret : -ret;
}


void ADC_callbackDRDY ()
{
  if (adc.state & ADC_FIRST_READ) {
    // Clear first read
    adc.state &= ~ADC_FIRST_READ;
    ADC_sample_();
  }
  if (adc.state & ADC_READY) {
    ADC_sample_();
  }
}
