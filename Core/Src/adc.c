//
// Created by devsploit on 2020/08/17.
//

#include "adc.h"
#include "logger.h"

static inline uint32_t ADC_bswap32_24_ (uint8_t frame[3]);

void ADC_sample_ ();

ADC_t adc = {0};

int ADC_init (SPI_HandleTypeDef *interface)
{
  adc.spi   = interface;
  adc.state = ADC_IDLE; // Block interrupts

  adc.wav.fname         = "REC";
  adc.wav.sampleRate    = 4000; // sps
  adc.wav.nChannels     = 4;
  adc.wav.bitsPerSample = 24;
  adc.wav.blockSize     = 32; // bits

  WAVE_createFile(&adc.wav);

  if (ADC_CS_IS_ENABLE()) {
    WARN("ADC CS low (unknown state)");
    ADC_CS_DISABLE();
    HAL_Delay(ADC_WAIT_DELTA_);
  }

  ADC_sendCommand(ADC_CMD_OP_RESET);
  DBUG("Reset: 0x%06X", ADC_sendCommand(ADC_CMD_OP_NULL));

  ADC_sendCommand(ADC_CMD_OP_RREG | ADC_ADR_ID);
  DBUG("ID: 0x%06X", ADC_sendCommand(ADC_CMD_OP_NULL));

  while (!ADC_IS_READY());


  adc.state = ADC_READY | ADC_FIRST_READ;
  return 1;
}

int ADC_yield ()
{
  if (adc.storePtr != NULL) {
    WAVE_appendData(&adc.wav, adc.storePtr, ADC_RX_LEN / 2 * sizeof(uint32_t), 1);
    DBUG("Persisting ADC buffer");
    adc.storePtr = NULL;
  }
}

static inline uint32_t ADC_bswap32_24_ (uint8_t frame[3])
{
  return (unsigned) (frame[0] << 16u)
         | (unsigned) (frame[1] << 8u)
         | (unsigned) frame[2];
}

void ADC_sample_ ()
{
  ADC_CS_ENABLE();
  HAL_SPI_Receive(adc.spi, (uint8_t *) adc.spiRx, (ADC_FRAME_NUM * ADC_FRAME_SIZE), ADC_TIMEOUT);
  ADC_CS_DISABLE();

  // Convert from Big Endian to Little Endian and persist
  for (size_t i = 0; i < ADC_NUM_CH; i++) {
    adc.rx[adc.rxPos][i] = ADC_bswap32_24_(adc.spiRx[i + ADC_RESPONSE_LEN]);
  }

  // Increment counters
  adc.rxPos++;
  adc.sampleCount++;

  // Half Complete
  if (adc.rxPos == ADC_RX_LEN / 2) {
    adc.storePtr = (uint32_t *) adc.rx;
  }
    // Full Complete
  else if (adc.rxPos == ADC_RX_LEN) {
    adc.storePtr = (uint32_t *) adc.rx[ADC_RX_LEN / 2];
    adc.rxPos    = 0;
  }

}

int ADC_sendCommand (uint16_t cmd)
{
  HAL_StatusTypeDef status;
  // Little Endian to Big Endian with zero padding
  uint8_t           tx[ADC_FRAME_NUM][ADC_FRAME_SIZE] = {{cmd >> 8u, cmd & 0xFFu}};

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
