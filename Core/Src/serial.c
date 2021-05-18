/**
  ******************************************************************************
  * @file     lib.c
  * @author   Auto-generated by STM32CubeIDE
  * @version  V1.0
  * @date     31/07/2020 09:04:15
  * @brief    Default under dev library file.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "serial.h"
#include <assert.h>

/** Functions ----------------------------------------------------------------*/
// Private Function Defs.
int Serial_initRx_ (Serial_t *self, UART_HandleTypeDef *uart);

int Serial_initTx_ (Serial_t *self, UART_HandleTypeDef *uart);

int Serial_wrap (Serial_t *self, UART_HandleTypeDef *uart)
{
  self->config_.uart = uart;

  if (Serial_initRx_(self, uart) > 0
      && Serial_initTx_(self, uart) > 0) {
    return 1;
  }
  else {
    return 0;
  }
}

size_t Serial_available (Serial_t *self)
{
  // TODO: Implement UART IDLE detection
  uint32_t dmaHead        = self->config_.uart->hdmarx->Instance->NDTR;
  uint32_t readHead       = self->rx.idx;
  int32_t bytesAvailable = SERIAL_RX_LEN - readHead - dmaHead;
  assert(bytesAvailable <= SERIAL_RX_LEN);

  return bytesAvailable >= 0 ? bytesAvailable : bytesAvailable + SERIAL_RX_LEN;
}

uint8_t Serial_read (Serial_t *self)
{
#ifdef SERIAL_RX_DMA
  // Get current val form buffer
  uint8_t *ret = Serial_tail(self);
  // Mark as read
  // Cheap fix for circ buff
  Serial_advanceTailRx(self, 1);
  return *ret;
#else
#error Operation not supported
#endif
}

uint8_t Serial_peek (Serial_t *self)
{
#ifdef SERIAL_RX_DMA
  return *(Serial_tail(self));
#else
#error Operation not supported
#endif

}

int Serial_write (Serial_t *self, const uint8_t *buf, size_t len)
{
#ifdef SERIAL_TX_BLOCKING
  // NOTE: This is a little pointer hack to allow for `const` buffers to be
  // passed making the overall program more memory efficient. This is based on
  // a brief code review of STM's UART transmit functions. Future changes to
  // this function might break some things.
  if (HAL_UART_Transmit(self->config_.uart, (uint8_t *) buf, len, SERIAL_MAX_DELAY) == HAL_OK)
    return 1;
  else
    return 0;
#else
#error Operation not supported
#endif
}

int Serial_println (Serial_t *self, const char *str)
{
  int n = Serial_print(self, str);
  n += Serial_print(self, SERIAL_EOL);
  return n;
}

int Serial_print (Serial_t *self, const char *str)
{
  return Serial_write(self, (uint8_t *) str, strlen(str));
}

int Serial_initRx_ (Serial_t *self, UART_HandleTypeDef *uart)
{
#ifdef SERIAL_RX_DMA
  self->rx.idx = 0;
  if (HAL_UART_Receive_DMA(uart, self->rx.buf_, SERIAL_RX_LEN) == HAL_OK)
    return 1;
  else
    return 0;
#else
#error Operation not supported
#endif
}

int Serial_initTx_ (Serial_t *self, UART_HandleTypeDef *uart)
{
#ifdef SERIAL_TX_BLOCKING
  return 1;
#else
#error Operation not supported
#endif
}
