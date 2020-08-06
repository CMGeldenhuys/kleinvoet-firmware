//
// Created by CM Geldenhuys on 2020/08/06.
//

#ifndef LOGGER_H_
#define LOGGER_H_

#include <stdarg.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

#define DBUG(msg, ...) LOG_log(__func__, LOG_DEBUG, (msg), ##__VA_ARGS__)
#define INFO(msg, ...) LOG_log(__func__, LOG_INFO, (msg) ##__VA_ARGS__)
#define WARN(msg, ...) LOG_log(__func__, LOG_WARN, (msg) ##__VA_ARGS__)
#define ERR(msg, ...) LOG_log(__func__, LOG_ERR, (msg) ##__VA_ARGS__)

#ifndef LOG_BUF_LEN
  #define LOG_BUF_LEN 64
#endif

#ifndef LOG_MSG_LEN
  #define LOG_MSG_LEN 40
#endif

#define LOG_MSG_INFO_LEN sizeof("[LEVL:func_name] YYYY-MM-DD HH:mm:ss - ")

typedef enum {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERR
} LOG_Lvl_e;



int LOG_log(const char *funcName, LOG_Lvl_e lvl, char *format, ...);
int LOG_write(uint8_t *buf, size_t len);
int LOG_flush();


#endif //FIRMWARE_LOGGER_H
