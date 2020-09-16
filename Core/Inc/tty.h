/*
 * tty.h
 *
 *  Created on: Jul 31, 2020
 *      Author: CM Geldenhuys
 */

#ifndef TTY_H_
#define TTY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "serial.h"

#ifndef TTY_SERVICE_INTERVAL
#define TTY_SERVICE_INTERVAL 1000
#endif

#ifndef TTY_CMD_BUF
#define TTY_CMD_BUF 128
#endif

#ifndef TTY_CMD_LST
#define TTY_CMD_LST 16
#endif

#ifndef TTY_SCREEN_WIDTH
#define TTY_SCREEN_WIDTH 80
#endif

#ifndef TTY_ARGS_SPLIT_TOKEN
#define TTY_ARGS_SPLIT_TOKEN " "
#endif

#ifndef TTY_ARGS_LEN
#define TTY_ARGS_LEN 4
#endif

#ifndef TTY_EOL
#define TTY_EOL SERIAL_EOL
#endif

#define TTY_VT100_CLEAR_LINE_RIGHT "\e[K"
#define TTY_VT100_CLEAR_SCREEN "\e[2J"

typedef struct {
    const char *name;

    int (*func) (int argc, char *argv[]);
} TTY_Command_t;

typedef struct {
    char *PS;
    uint32_t tick;
    struct {
        uint8_t buffer[TTY_CMD_BUF];
        uint8_t *pos;
        size_t len;
    }        command;

    TTY_Command_t commandList[TTY_CMD_LST];

    Serial_t *serial;
} TTY_t;

extern TTY_t tty;

int TTY_init (UART_HandleTypeDef *uart);

int TTY_yield ();

int TTY_registerCommand (const char *command, int (*func) (int argc, char *argv[]));

int TTY_println (const char *str);

int TTY_print (const char *str);

int TTY_write (uint8_t *buf, size_t len);

int TTY_available ();

uint8_t TTY_read ();

int CMD_resetDevice (int argc, char **argv);

// TODO: Find fix for linker to pick up
#define TTY_PRINTF
#ifdef TTY_PRINTF

int TTY_printf (const char *fmt, ...);

#endif

#ifdef __cplusplus
}
#endif

#endif /* TTY_H_ */
