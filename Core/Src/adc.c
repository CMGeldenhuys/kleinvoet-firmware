//
// Created by devsploit on 2020/08/17.
//

#include "adc.h"
#include "logger.h"

int ADC_TxRx_ (uint8_t *txBuf, uint8_t *prevRxBuf, size_t len);

//int ADC_Tx_ (uint16_t *buf, size_t len);

ADC_t adc = {0};

int ADC_init (SPI_HandleTypeDef *interface)
{
  adc.spi   = interface;
  adc.state = ADC_IDLE; // Block interrupts

  if (ADC_CS_IS_ENABLE()) {
    WARN("ADC CS low (unknown state)");
    ADC_CS_DISABLE();
    HAL_Delay(ADC_WAIT_DELTA_);
  }

  // Reset device && make sure device is ready
  uint32_t wait                              = 0;
  uint8_t  rx[ADC_FRAME_NUM][ADC_FRAME_SIZE] = {0};

  ADC_sendCommand(ADC_CMD_OP_RESET, rx);

  while ((!ADC_IS_READY()
          || (rx[0][0] != 0xFF || rx[0][1] != 0x24))
         && (wait += ADC_WAIT_DELTA_) < ADC_TIMEOUT) {
    ADC_sendCommand(ADC_CMD_OP_RESET, rx);
    // Use logging as delay
    WARN("Device not ready...(%u - 0x%04X)", wait, ADC_BYTE_CAT(rx[0][0], rx[0][1]));
  }

  if (wait > ADC_TIMEOUT) return -1;

  // Wait till device ready or timed out
  while (!ADC_IS_READY()
         && (wait += ADC_WAIT_DELTA_) < ADC_TIMEOUT);

  // Clear first sample from buffer
  ADC_sendCommand(ADC_CMD_OP_NULL, NULL);
  adc.state = ADC_READY;

}

int ADC_sendCommand (uint16_t cmd, uint8_t rx[ADC_FRAME_NUM][ADC_FRAME_SIZE])
{
  uint8_t tx[ADC_FRAME_NUM][ADC_FRAME_SIZE] = {{ADC_BYTE_1(cmd), ADC_BYTE_0(cmd)}};
  return ADC_TxRx_((uint8_t *) tx, (uint8_t *) rx, ADC_FRAME_LEN);
}


int ADC_TxRx_ (uint8_t *txBuf, uint8_t *prevRxBuf, size_t len)
{
  HAL_StatusTypeDef ret;
  ADC_CS_ENABLE();

  // Only read
  if (txBuf == NULL) {
    ret = HAL_SPI_Receive(adc.spi, prevRxBuf, len, ADC_TIMEOUT);
  }
    // Ony write
  else if (prevRxBuf == NULL) {
    ret = HAL_SPI_Transmit(adc.spi, txBuf, len, ADC_TIMEOUT);
  }
    // Read and Write
  else {
    ret = HAL_SPI_TransmitReceive(adc.spi, txBuf, prevRxBuf, len, ADC_TIMEOUT);
  }

  ADC_CS_DISABLE();
  return ret == HAL_OK;
}

void ADC_callbackDRDY ()
{
  if (adc.state & ADC_READY) {
    // TODO: do more efficiently
    if (!ADC_sendCommand(ADC_CMD_OP_NULL, NULL)) ERR("ADC comm fail!");
  }
}
