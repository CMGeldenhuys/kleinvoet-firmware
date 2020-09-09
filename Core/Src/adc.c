//
// Created by devsploit on 2020/08/17.
//

#include "adc.h"
#include "logger.h"

int ADC_TxRx_ (uint16_t *txBuf, uint16_t *prevRxBuf, size_t len);

//int ADC_Tx_ (uint16_t *buf, size_t len);

ADC_t adc = {0};

int ADC_init (SPI_HandleTypeDef *interface)
{
  adc.spi   = interface;
  adc.state = ADC_IDLE;

  if (ADC_CS_IS_ENABLE()) {
    WARN("ADC CS low (unknown state)");
    ADC_CS_DISABLE();
    HAL_Delay(ADC_WAIT_DELTA_);
  }

  // Reset device && make sure device is ready
  uint32_t wait = 0;
  uint16_t rx   = 0;

//  while ((!ADC_IS_READY()
//          || rx != 0xff24)
//         && (wait += ADC_WAIT_DELTA_) < ADC_TIMEOUT) {
//    rx = ADC_sendCommand(ADC_CMD_OP_RESET);
//    // Use logging as delay
//    INFO("Device not ready...(%u - 0x%02X)", wait, rx);
//  }
  ADC_sendCommand(ADC_CMD_OP_RESET);
  // Wait till device ready
  while(!ADC_IS_READY());
  // Clear first sample from buffer
  ADC_sendCommand(ADC_CMD_OP_NULL);
  adc.state = ADC_READY;
//  // NOTE: After ADC ready you have 1/SR before ADC produces an interrupt
//  adc.state = ADC_FIRST_READ;
//
//  WARN("Waiting till ADC first read");
//  while (adc.state != ADC_READY);
//  ADC_sendCommand(ADC_CMD_OP_NULL);
//  INFO("ADC ready!");

  return wait < ADC_TIMEOUT;
}

int32_t ADC_sendCommand (uint16_t cmd)
{
  uint16_t tx[ADC_FRAME_LENGTH] = {cmd};
  uint16_t rx[ADC_FRAME_LENGTH] = {0};
//  ADC_Tx_(tx, ADC_FRAME_LENGTH);
//  ADC_Rx_(rx, ADC_FRAME_LENGTH);
  ADC_TxRx_(tx, rx, ADC_FRAME_LENGTH);

  // TODO: Handle failed TX or RX

  return *rx;
}


int ADC_TxRx_ (uint16_t *txBuf, uint16_t *prevRxBuf, size_t len)
{
//  HAL_StatusTypeDef ret;
  uint8_t tmpTx = 0x0;
  uint8_t tmpRx = 0x0;

  // Account for SPI hardware frame size

  ADC_CS_ENABLE();

  for (int i = 0; i < len; i++) {
    // BYTE 0 (little endian)
    HAL_SPI_TransmitReceive(adc.spi, (uint8_t *)txBuf + i + 1, (uint8_t *)prevRxBuf + i + 1, 1, ADC_TIMEOUT);

    // BYTE 1
    HAL_SPI_TransmitReceive(adc.spi, (uint8_t *)txBuf + i, (uint8_t *)prevRxBuf + i, 1, ADC_TIMEOUT);

    // BYTE 2
    HAL_SPI_TransmitReceive(adc.spi, &tmpTx, &tmpRx, 1, ADC_TIMEOUT);
  }
//
//  ret = HAL_SPI_TransmitReceive(adc.spi,
//                                (uint8_t *) txBuf, (uint8_t *) prevRxBuf,
//                                len, ADC_TIMEOUT);
  ADC_CS_DISABLE();

  return 1;
//  return ret == HAL_OK;
}

//int ADC_Tx_ (uint16_t *buf, size_t len)
//{
//  HAL_StatusTypeDef ret;
//
//  // Account for SPI hardware frame size
//
//  ADC_CS_ENABLE();
//  ret = HAL_SPI_Transmit(adc.spi,
//                         (uint8_t *) buf,
//                         len, ADC_TIMEOUT);
//  ADC_CS_DISABLE();
//
//  return ret == HAL_OK;
//}

uint16_t tmp[ADC_FRAME_LENGTH] = {ADC_CMD_OP_NULL};

void ADC_callbackDRDY ()
{
//  if (adc.state & ADC_FIRST_READ) {
//    if(!ADC_sendCommand(ADC_CMD_OP_NULL)) ERR("ADC comm fail!");
//    DBUG("First read");
////    HAL_SPI_Receive_DMA(adc.spi, (uint8_t *) adc.buf, ADC_BUF_LEN);
////    DBUG("DMA Setup");
//    adc.state = ADC_READY;
//  }

  if (adc.state & ADC_READY) {
    HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    // TODO: do more efficiently
    if(!ADC_sendCommand(ADC_CMD_OP_NULL)) ERR("ADC comm fail!");
  }
}
