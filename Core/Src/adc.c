//
// Created by devsploit on 2020/08/17.
//

#include "adc.h"
#include "logger.h"

int ADC_TxRx_ (uint32_t *txBuf, uint32_t *prevRxBuf, size_t len);

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

  DBUG("RESET: 0x%04X", ADC_sendCommand(ADC_CMD_OP_RESET));
  DBUG("RESET: 0x%04X", ADC_sendCommand(ADC_CMD_OP_RESET));

  while (!ADC_IS_READY());

  DBUG("ID: 0x%04X", ADC_sendCommand(ADC_CMD_OP_RREG | ADC_ADR_ID));
  DBUG("STATUS: 0x%04X", ADC_sendCommand(ADC_CMD_OP_RREG | ADC_ADR_STATUS));
  DBUG("NOP: 0x%04X", ADC_sendCommand(ADC_CMD_OP_NULL));

//  // Reset device && make sure device is ready
//  uint32_t wait = 0;
//  uint8_t rx[ADC_FRAME_NUM][ADC_FRAME_SIZE] = {0};
//
//
//  // Reset ADC to known state
//  ADC_sendCommand(ADC_CMD_OP_RESET, rx);
//
//  while ((!ADC_IS_READY()
//          || (rx[0][0] != 0xFF || rx[0][1] != 0x24))
//         && (wait += ADC_WAIT_DELTA_) < ADC_TIMEOUT) {
//    ADC_sendCommand(ADC_CMD_OP_RESET, rx);
//    // Use logging as delay
//    WARN("Device not ready...(%u - 0x%04X)", wait, ADC_BYTE_CAT(rx[0][0], rx[0][1]));
//  }
//
//  if (wait > ADC_TIMEOUT) return -1;

  //
//  ADC_sendCommand(ADC_CMD_OP_WREG
//  | ADC_ADR_MODE
//  | ADC_MODE_RESET_ACK
//  |)

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

void ADC_sample_ ()
{
  uint32_t rx[ADC_FRAME_NUM] = {0};
  if (ADC_TxRx_(NULL, rx, ADC_FRAME_LEN) > 0) {
    // Store Sample (ignore status response)
    memcpy(adc.rx[adc.rxPos], rx + 1, ADC_NUM_CH * sizeof(uint32_t));
  }
  else {
    // Coms failed so store 0xFF instead
    memset(adc.rx[adc.rxPos], 0x00, ADC_NUM_CH * sizeof(uint32_t));
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
  uint32_t rx[ADC_FRAME_NUM] = {0};
  uint32_t tx[ADC_FRAME_NUM] = {cmd << 8U};

  if (ADC_TxRx_(tx, rx, ADC_FRAME_NUM) > 0) return *rx >> 8U; // Result of CMD is only 16-bits
  else return -1;
}

int ADC_TxRx_ (uint32_t *txBuf, uint32_t *prevRxBuf, size_t len)
{
  HAL_StatusTypeDef ret;
  uint32_t          tx = 0, rx = 0;

  // Select chip
  ADC_CS_ENABLE();
  for (size_t i = 0; i < len; i++) {
    // Some transmission involved
    if (txBuf != NULL) {
      // Convert to big endian (swap bytes)
      tx = __builtin_bswap32(txBuf[i]);
    }

    // Send out one frame
    ret = HAL_SPI_TransmitReceive(adc.spi,              // Send to ADC
                                  (uint8_t *) (&tx) + 1,    // Only send last 3 bytes (24-bit)
                                  (uint8_t *) (&rx),    // Rx -> big endian
                                  ADC_FRAME_LEN,        // Frame Size: 3 Bytes
                                  ADC_TIMEOUT);

    // If transaction failed
    if (ret != HAL_OK) {
      ADC_CS_DISABLE();
      // STM32 HAL bug leaves line IDLE last bit state
      HAL_GPIO_WritePin(ADC_MOSI_GPIO_Port, ADC_MOSI_Pin, GPIO_PIN_SET);
      return 0;
    }

    // Care about returned values.
    if (prevRxBuf != NULL) {
      // Convert to little endian & shift one byte down
      prevRxBuf[i] = __builtin_bswap32(rx) >> 8U;
    }
  }
  ADC_CS_DISABLE();
  // STM32 HAL bug leaves line IDLE last bit state
  HAL_GPIO_WritePin(ADC_MOSI_GPIO_Port, ADC_MOSI_Pin, GPIO_PIN_SET);
  return 1;
}

void ADC_callbackDRDY ()
{
  if (adc.state & ADC_FIRST_READ) {
    adc.state &= ~ADC_FIRST_READ;
    ADC_sendCommand(ADC_CMD_OP_NULL);
  }
  if (adc.state & ADC_READY) {
    // TODO: do more efficiently
    ADC_sample_();
  }
}
