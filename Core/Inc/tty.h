/**
 *
 * @file tty.h
 * @author CM Geldenhuys
 * @date 31 Jul. 2020
 *
 * @brief TTY prompt for basic interactions with the MCU using a REPL
 *
 *
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
#define TTY_VT100_COLOR_DEFAULT "\e[39m"

#define STM32_UUID ((uint32_t *)0x1FFF7A10)

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

typedef int (*REPL_cmd_t) (int argc, char *argv[]);

/**
 * @brief Initialise TTY with handle to serial interface
 *
 * @param [in] uart HAL serial handle
 *
 * @return Returns a value greater than 0 if successful
 */
int TTY_init (UART_HandleTypeDef *uart);

/**
 * @brief Runs current event loop
 *
 * @return Returns a value greater than 0 if successful
 */
int TTY_yield ();

/**
 * @brief Registers a new command with the REPL
 *
 * @param [in] command Command name or alias
 * @param [in] func Function to be executed by REPL
 *
 * @return Returns a value greater than 0 if successful
 */
int TTY_registerCommand (const char *alias, REPL_cmd_t func);

/**
 * @brief Prints string to attached serial with newline ending
 *
 * @see Serial_println(Serial_t *, const char *)
 *
 * @param [in] str String message to print
 *
 * @return Returns a value greater than 0 if successful
 */
int TTY_println (const char *str);

/**
 * @brief Prints string to attached serial
 *
 * @see Serial_print(Serial_t *, const char *)
 *
 * @param [in] str String message to print
 *
 * @return Returns a value greater than 0 if successful
 */
int TTY_print (const char *str);

int TTY_write (uint8_t *buf, size_t len);

/**
 * @brief Returns number of bytes available on serial interface
 *
 * @see Serial_available(Serial_t *)
 *
 * @return Returns a value greater than 0 if successful
 */
size_t TTY_available ();

/**
 * @brief Reads current byte from serial interface
 *
 * @see Serial_read(Serial_t *)
 *
 * @return Returns a value greater than 0 if successful
 */
uint8_t TTY_read ();

/**
 * @brief REPL command to reset device
 * @param [in] argc Unused
 * @param [in] argv Unused
 *
 * @return Does not return
 */
int CMD_resetDevice (int argc, char **argv);

// TODO: Find fix for linker to pick up
#define TTY_PRINTF
#ifdef TTY_PRINTF

/**
 * @brief Format printing to serial interface
 *
 * @param [in] fmt String format
 * @param [in] ... Arguments passed to formatter
 *
 * @return Returns a value greater than 0 if successful
 */
int TTY_printf (const char *fmt, ...);

#endif

#ifdef __cplusplus
}
#endif

#endif /* TTY_H_ */
