//
// Created by CM Geldenhuys on 2021/02/15.
//

#ifndef FIRMWARE_TIMESTAMP_H
#define FIRMWARE_TIMESTAMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "fifofast.h"
#include "gps.h"

#ifndef TIME_FILENAME
#define TIME_FILENAME "TS.CSV"
#endif

#ifndef TIME_MARK_LEN
#define TIME_MARK_LEN 16
#endif

#define TIME_TIMEDATE_FORMAT "%4u-%02u-%02u %02u:%02u:%02u.%lu"

typedef struct {
    size_t     sample;
    const char *comment;
} metastamp_t;

typedef struct {
    const TIM_HandleTypeDef *tim;
    FIL                     *fp;
    uint_least8_t           timeLocked;
    struct {
        uint_least8_t flushFile;
    }                       schedule;
    struct {
        _fff_declare(uint32_t, timestamp, TIME_MARK_LEN);
        _fff_declare(metastamp_t, metastamp, TIME_MARK_LEN);
    }                       fifo;

} timestamp_t;

int TIME_init (const TIM_HandleTypeDef *tim);

void TIME_stamp (const UBX_NAV_TIMEUTC_t *cmd);

void TIME_mark ();

void TIME_timeValid ();

void TIME_timeInvalid ();

void TIME_meta (const char *const comment);

void TIME_flush (uint_least8_t force);

void TIME_yield ();


#ifdef __cplusplus
}
#endif
#endif //FIRMWARE_TIMESTAMP_H
