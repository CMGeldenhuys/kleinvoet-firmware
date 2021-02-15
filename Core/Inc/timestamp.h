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
#define TIME_MARK_LEN 4
#endif

#define TIME_TIMEDATE_FORMAT "%4u-%02u-%02u %02u:%02u:%02u.%lu"

typedef struct {
    const TIM_HandleTypeDef *tim;
    FIL                     *fp;
    uint_least8_t           timeLocked;
    _fff_declare(uint32_t, fifo, TIME_MARK_LEN);
} timestamp_t;

int TIME_init (const TIM_HandleTypeDef *tim);

void TIME_stamp (const UBX_NAV_TIMEUTC_t *cmd);

void TIME_mark ();

void TIME_timeValid ();

void TIME_timeInvalid ();


#ifdef __cplusplus
}
#endif
#endif //FIRMWARE_TIMESTAMP_H
