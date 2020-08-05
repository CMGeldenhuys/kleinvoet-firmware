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

/** Functions ----------------------------------------------------------------*/
// Private Function Defs.
int Serial_initRx_(Serial_t* self, UART_HandleTypeDef* uart);
int Serial_initTx_(Serial_t* self, UART_HandleTypeDef* uart);

int Serial_wrap(Serial_t* self, UART_HandleTypeDef* uart)
{
	self->__config.uart = uart;

	if(Serial_initRx_(self, uart) > 0
     && Serial_initTx_(self, uart) > 0 ) {
		return 1;
	}
	else {
		return 0;
	}
}

int Serial_available(Serial_t* self)
{
	return SERIAL_RX_LEN - self->rx.idx - self->__config.uart->hdmarx->Instance->NDTR;
}

uint8_t Serial_read(Serial_t* self)
{
#ifdef SERIAL_RX_DMA
	uint8_t* ret = self->rx.__buf + self->rx.idx;
	self->rx.idx++;
	self->rx.idx -= self->rx.idx != SERIAL_RX_LEN ? 0 : SERIAL_RX_LEN;
	return *ret;
#else
#error Operation not supported
#endif
}

uint8_t Serial_peek(Serial_t* self)
{
#ifdef SERIAL_RX_DMA
	return *(self->rx.__buf + self->rx.idx);
#else
#error Operation not supported
#endif

}

int Serial_write(Serial_t* self, uint8_t* buf, const size_t len)
{
#ifdef SERIAL_TX_BLOCKING
	if(HAL_UART_Transmit(self->__config.uart, buf, len, SERIAL_MAX_DELAY) == HAL_OK)
		return 1;
	else
		return 0;
#else
#error Operation not supported
#endif
}

int Serial_println(Serial_t* self, const char* str)
{
	int n = Serial_print(self, str);
	n += Serial_print(self, SERIAL_EOL);
	return n;
}

int Serial_print(Serial_t* self, const char* str)
{
	return Serial_write(self, (uint8_t*)str, strlen(str));
}

int Serial_initRx_(Serial_t* self, UART_HandleTypeDef* uart)
{
#ifdef SERIAL_RX_DMA
	self->rx.idx = 0;
	if(HAL_UART_Receive_DMA(uart, self->rx.__buf, SERIAL_RX_LEN) == HAL_OK)
		return 1;
	else
		return 0;
#else
#error Operation not supported
#endif
}

int Serial_initTx_(Serial_t* self, UART_HandleTypeDef* uart)
{
#ifdef SERIAL_TX_BLOCKING
	return 1;
#else
#error Operation not supported
#endif
}
