/*
 * serial.h
 *
 *  Created on: Jul 30, 2020
 *      Author: CM Geldenhuys
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"

#ifndef SERIAL_MAX_DELAY
#define SERIAL_MAX_DELAY HAL_MAX_DELAY
#endif

#define SERIAL_CRLF "\r\n"
#define SERIAL_BS "\b"

#ifndef SERIAL_EOL
#define SERIAL_EOL SERIAL_CRLF
#endif

#ifndef SERIAL_BUF_LEN
#define SERIAL_BUF_LEN 128
#endif

#ifndef SERIAL_RX_LEN
#define SERIAL_RX_LEN SERIAL_BUF_LEN
#endif

#ifndef SERIAL_TX_LEN
#define SERIAL_TX_LEN SERIAL_BUF_LEN
#endif

#if !defined(SERIAL_RX_DMA) || !defined(SERIAL_RX_BLOCKING) || !defined(SERIAL_RX_INTERRUPT)
#define SERIAL_RX_DMA
#endif

#if !defined(SERIAL_TX_DMA) || !defined(SERIAL_TX_BLOCKING) || !defined(SERIAL_TX_INTERRUPT)
#define SERIAL_TX_BLOCKING
#endif

typedef struct {
	struct {
		UART_HandleTypeDef *uart;
	} __config;

	struct {
		uint8_t __buf[SERIAL_RX_LEN];
		size_t idx;
	} rx;

	struct {
		uint8_t __buf[SERIAL_TX_LEN];
		size_t idx;
	} tx;

} Serial_t;

// Public Functions
int Serial_wrap(Serial_t* self, UART_HandleTypeDef* uart);
int Serial_available(Serial_t* self);
uint8_t Serial_read(Serial_t* self);
int Serial_write(Serial_t* self, uint8_t* buf, const size_t len);
int Serial_println(Serial_t* self, const char* str);
int Serial_print(Serial_t* self, const char* str);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H_ */
