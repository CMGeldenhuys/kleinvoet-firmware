//
// Created by CM Geldenhuys on 2021/02/15.
//

#include "timestamp.h"

// NB: see _fff_init for initialisation
// fifo = {0, 0, 0, {}}
volatile timestamp_t ts = {0};

// Declare new FIFO
_fff_declare(uint32_t, fifo_uint32, TIME_MARK_LEN);

int TIME_init (const TIM_HandleTypeDef *tim)
{
  // Store ref to timer to keep track of ADC frames
  ts.tim = tim;

  // Create ts file for keeping track of timestamping and other meta data
  DBUG("Allocating memory for timestamping file");
  ts.fp = FATFS_malloc(0);

  // Check if file is valid
  if (ts.fp == NULL) return -2;

  // Create and open file
  DBUG("Open/creating file");
  if (FATFS_open(
          ts.fp,
          TIME_FILENAME,
          FA_CREATE_ALWAYS | FA_WRITE) <= 0)
    return -2;

  // Write header of CSV file
  DBUG("Writing header to file");
  f_printf(ts.fp, "\"Sample\",\"Date/Time\",\"Time Accuracy\",\"Comments\"" FATFS_EOL);

  // Sync filesystem
  FATFS_sync(ts.fp);

  // Success
  return 1;
}

__attribute__((always_inline))
inline void TIME_timeValid ()
{
  ts.timeLocked = 1;
}

__attribute__((always_inline))
inline void TIME_timeInvalid ()
{
  ts.timeLocked = 0;
}

__attribute__((always_inline))
inline void TIME_mark ()
{
  if (ts.timeLocked) {
    DBUG("new sample marked");
    const uint32_t sample = ts.tim->Instance->CNT;
    if (_fff_is_full((ts.fifo))) {
      WARN("FIFO full, timestamp missed!");
    }
    _fff_write((ts.fifo), sample);
  }
}

__attribute__((always_inline))
inline void TIME_stamp (const UBX_NAV_TIMEUTC_t *cmd)
{
  DBUG("new sample stamped");
  if (_fff_is_empty((ts.fifo))) {
    WARN("FIFO empty, no sample to mark...");
  }
  else {
    const uint32_t sample = _fff_read((ts.fifo));
    f_printf(ts.fp, "%lu,%4u-%02u-%02u %02u:%02u:%02u.%lu,%lu,\"%s\"" FATFS_EOL,
             sample,
             cmd->year, cmd->month, cmd->day,
             cmd->hour, cmd->min, cmd->sec, cmd->nano,
             cmd->tAcc, "");
  }

}