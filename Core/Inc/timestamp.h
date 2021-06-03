/**
 *
 * @file timestamp.h
 * @author CM Geldenhuys
 * @date 15 Feb. 2021
 *
 * @brief Timestamping metadata on a per sample basis using a timer module to
 * track samples
 *
 * Timer module is driven by the ADC's frame clock to keep track of the current
 * frame. This information is combined with time code information sent by the
 * GPS module. THis allows one to timestamp on a per frame basis. Since the GPS
 * needs time to compute internally compute the latest timing information; a
 * sample is first marked and then stamped. The abstraction also offers an API
 * to stamp arbitrary samples with meta data.
 *
 */

#ifndef FIRMWARE_TIMESTAMP_H
#define FIRMWARE_TIMESTAMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "fifofast.h"
#include "gps.h"

/// Name of timestamping file
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

/**
 * @brief Initialise timestamping functionality
 *
 * Create necessary files to store the timestamping information and configure
 * internal states.
 *
 * @param [in] tim Pointer to timer module used to track frame count
 *
 * @return Returns a value greater than 1 if successful.
 */
int TIME_init (const TIM_HandleTypeDef *tim);

/**
 * @brief Time stamp associated marked sample
 *
 * @param [in] cmd UBX time command received from GPS
 *
 * @note Information will not be persisted/processed till next call to
 * `TIME_yield()`
 */
void TIME_stamp (const UBX_NAV_TIMEUTC_t *cmd);

/**
 * @brief Store refrence to current frame count that will be associated with the
 * next call to `TIME_stamp`.
 *
 * @see TIME_stamp()
 */
void TIME_mark ();

/**
 * @brief Notify library that the current timing information is valid and that
 * calls to timestamping may be processed.
 *
 * @note If the timing information is not valid the stamping information will
 * not be persisted.
 */
void TIME_timeValid ();

/**
 * @brief Notify library that current timing information is invalid.
 *
 * @see TIME_timeValid()
 */
void TIME_timeInvalid ();

/**
 * @brief Attach metadata to current sample with the given comment
 *
 * @param comment
 */
void TIME_meta (const char *const comment);

/**
 * @brief Flush any changes and cached filesystem data to SD card
 *
 * @param [in] force Do not wait for next yield
 *
 * @note Filesystem will only flush on next call to yield unless `force` param
 * is passed. This is to prevent any race conditions if called from within an
 * interrupt.
 */
void TIME_flush (uint_least8_t force);

/**
 * @brief Called from main loop, runs scheduled tasks and processes internal
 * state machine
 */
void TIME_yield ();


#ifdef __cplusplus
}
#endif
#endif //FIRMWARE_TIMESTAMP_H
