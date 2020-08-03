/*
 * tty.h
 *
 *  Created on: Jul 31, 2020
 *      Author: CM Geldenhuys
 */

#ifndef TTY_H_
#define TTY_H_

#include "stm32f4xx_hal.h"
#include "serial.h"

#ifndef TTY_SERVICE_INTERVAL
#define TTY_SERVICE_INTERVAL 1000
#endif

#ifndef TTY_GREETING
#define TTY_GREETING "Hello, World!"
#endif

#ifndef TTY_CMD_BUF
#define TTY_CMD_BUF 128
#endif

#ifndef TTY_CMD_LST
#define TTY_CMD_LST 8
#endif

typedef struct {
	const char* command;
	int (*func)(int argc, char *argv[]);
}TTY_Command_t;

typedef struct {
	char * PS;
	uint32_t tick;
	struct {
		uint8_t buffer[TTY_CMD_BUF];
		uint8_t * pos;
		size_t len;
	}command;

	TTY_Command_t commandList[TTY_CMD_LST];

	Serial_t* serial;
}TTY_t;

TTY_t tty;

int TTY_init(UART_HandleTypeDef* uart);
int TTY_yield();
int TTY_registerCommand(const char* command, int (*func)(int argc, char *argv[]));

#endif /* TTY_H_ */
