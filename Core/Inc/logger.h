/**
 *
 * @file logger.h
 * @author CM Geldenhuys
 * @date 6 Aug. 2020
 *
 * @brief Logger library for easy compile time logging abstraction
 *
 * Basic logger library that allows for compile time optimisation of severity
 * logging. Allows for variable level of user formatting, calling function
 * logging and variable user message buffer size.
 *
 * @attention Data persistence is to be implemented by user code. See
 * LOG_write() and LOG_flush().
 *
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "config.h"

#if !defined(LOG_DEST_TTY) && !defined(LOG_DEST_FILE)
#error "Please specify log device using 'LOG_DEST_****'"
#endif

// Default logging level if not defined
#if !defined(LOG_LEVEL_DEBUG) && !defined(LOG_LEVEL_INFO) && !defined(LOG_LEVEL_WARN)
#if defined(DEBUG)
#define LOG_LEVEL_DEBUG
#else
#define LOG_LEVEL_INFO
#endif
#endif

#if defined(LOG_LEVEL_DEBUG)
#define DBUG(msg, ...) LOG_log(__func__, LOG_DEBUG, (msg), ##__VA_ARGS__)
#else
#define DBUG(cmd, ...)
#endif

#if defined(LOG_LEVEL_DEBUG) || defined(LOG_LEVEL_INFO)
#define INFO(msg, ...) LOG_log(__func__, LOG_INFO, (msg), ##__VA_ARGS__)
#else
#define INFO(cmd, ...)
#endif

#if defined(LOG_LEVEL_DEBUG) || defined(LOG_LEVEL_INFO) || defined(LOG_LEVEL_WARN)
#define WARN(msg, ...) LOG_log(__func__, LOG_WARN, (msg), ##__VA_ARGS__)
#else
#define WARN(cmd, ...)
#endif

#define ERR(msg, ...) LOG_log(__func__, LOG_ERR, (msg), ##__VA_ARGS__)

#ifndef LOG_BUF_LEN
#define LOG_BUF_LEN 64
#endif

/// Maximum user log message length.
#ifndef LOG_MSG_LEN
#define LOG_MSG_LEN 1024
#endif

#ifndef LOG_EOL
#define LOG_EOL "\r\n"
#endif

#if defined(LOG_DEST_TTY) && !defined(LOG_COLOR_DISABLE)
#define LOG_COLOR
#endif

#ifdef LOG_COLOR
#define LOG_COLOR_FG_DEFAULT "\e[39m"
#define LOG_COLOR_FG_BLUE "\e[34m"
#define LOG_COLOR_FG_YELLOW "\e[33m"
#define LOG_COLOR_FG_RED "\e[31m"
#define LOG_COLOR_RESET "\e[0m"

#define LOG_COLOR_LEN (sizeof(LOG_COLOR_FG_DEFAULT) + sizeof(LOG_COLOR_RESET) - 2)

#else
#define LOG_COLOR_LEN 0
#endif

// -1 to account for NULL termination
#define LOG_MSG_INFO_LEN \
    (sizeof("[LEVL:function_name___] YYYY-MM-DD HH:mm:ss.uuu - ") - 1 + LOG_COLOR_LEN)

/**
 * @brief Enumeration of Log severity levels
 */
typedef enum {
    LOG_DEBUG,   /**< Debug level of severity*/
    LOG_INFO,    /**< Info level of severity*/
    LOG_WARN,    /**< Warning level of severity*/
    LOG_ERR      /**< Error level of severity*/
} LOG_Lvl_e;

/**
 * @brief Log message function.
 *
 * Write a message to the log with a certain level, calling function and format.
 *
 * @param [in] funcName String constant that contains the name of the logging
 *                      function
 * @param [in] lvl Severity of log (level)
 * @param [in] fmt String constant format passed to sprintf
 * @param [in] ... Varargs are passed to sprintf function
 *
 * @return Returns status code based on function success
 */
int LOG_log (const char *funcName, LOG_Lvl_e lvl, char *format, ...);

/**
 * @brief Persist raw log message data.
 *
 * Callback function used to persist log data. Has *WEAK* attribute to allow
 * overriding of function.
 *
 * @see LOG_flush()
 *
 * @param [in] buf Pointer to byte array that is to be persisted
 * @param [in] len Sizeof buffer array
 *
 * @return Returns number of bytes written or negative value no fail.
 */
int LOG_write (uint8_t *buf, size_t len);

/**
 * @brief Force write cached/buffered writes.
 *
 * Callback to force buffered writer to flush all cached writes to persistent
 * storage.
 *
 * @see LOG_write()
 *
 * @note Function is called on an ERROR level log to ensure error is persisted
 * in case of system failure.
 *
 * @return Returns number of bytes written or negative value no fail.
 */
int LOG_flush ();

int LOG_init();


#endif //FIRMWARE_LOGGER_H
