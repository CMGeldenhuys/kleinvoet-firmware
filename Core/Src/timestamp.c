/**
 *
 * @file timestamp.c
 * @author CM Geldenhuys
 * @date 15 Feb. 2021
 *
 * @headerfile timestamp.h "timestamp.h"
 *
 */

#include "adc.h"
#include "timestamp.h"

// NB: see _fff_init for initialisation
// fifo = {0, 0, 0, {}}
volatile timestamp_t ts = {
        .tim = 0,
        .fp = 0,
        .timeLocked = 0,
        .schedule = {
                .flushFile = 0
        },
        .fifo = {
                .timestamp = {0, 0, 0, {}},
                .metastamp = {0, 0, 0, {}}
        }
};

int TIME_init (const TIM_HandleTypeDef *tim)
{
  // Store ref to timer to keep track of ADC frames
  ts.tim = tim;

  // Create ts file for keeping track of timestamping and other meta data
  DBUG("Allocating memory for timestamping file");
  ts.fp = FATFS_malloc(1);

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
  if (FATFS_sync(ts.fp) <= 0)
    WARN("Failed to sync filesystem!");

  // Success
  return 1;
}

__attribute__((always_inline))
inline void TIME_timeValid ()
{
  INFO("Time locked");
  _fff_reset((ts.fifo.timestamp));
  ts.timeLocked = 1;
}

__attribute__((always_inline))
inline void TIME_timeInvalid ()
{
  INFO("Time no longer valid");
  ts.timeLocked = 0;
}

__attribute__((always_inline))
inline void TIME_mark ()
{
  if (!ts.timeLocked) return;
  const uint32_t sample = ts.tim->Instance->CNT;
  if (_fff_is_full((ts.fifo.timestamp))) {
    WARN("FIFO full, timestamp missed!");
  }
  else {
    _fff_write((ts.fifo.timestamp), sample);
    DBUG("new sample marked (%d)", _fff_mem_free((ts.fifo.timestamp)));
  }
}

__attribute__((always_inline))
inline void TIME_stamp (const UBX_NAV_TIMEUTC_t *cmd)
{
  if (!ts.timeLocked) return;
#ifdef WAVE_MOCK_WRITES
  INFO("Mock writes enabled, ignoring timestamp");
#else
  if (_fff_is_empty((ts.fifo.timestamp))) {
    WARN("FIFO empty, no sample to mark...");
  }
  else {
    const uint32_t sample = _fff_read((ts.fifo.timestamp));
    // TODO: move to yield
    char date[] = "YYYY-MM-dd";
    char timestamp[] = "HH:MM.ss.000000000";
    sniprintf(date, sizeof(date), "%4u-%02u-%02u",
              cmd->year, cmd->month, cmd->day);
    sniprintf(timestamp, sizeof(timestamp), "%02u:%02u:%02u.%09ld",
              cmd->hour, cmd->min, cmd->sec, cmd->nano);

    ADC_updateTimecode(date, timestamp, sample, cmd->tAcc);

    f_printf(ts.fp, "%lu,%sT%sZ,%lu,\"ts\"" FATFS_EOL,
             sample,
             date,
             timestamp,
             cmd->tAcc);
    DBUG("new sample stamped (%d)", _fff_mem_level((ts.fifo.timestamp)));
  }
#endif
}

__attribute__((always_inline))
inline void TIME_meta (const char *const comment)
{
  DBUG("Meta stamping file (%s)", comment);
  const metastamp_t meta = {.sample = ts.tim->Instance->CNT, .comment = comment};
  _fff_write((ts.fifo.metastamp), meta);
}

void TIME_flush (uint_least8_t force)
{
  // Check if there are changes to flush to FS
//  if (!ts.fileDirty) return;
  if(!force) {
    DBUG("Scheduled timestamp flush");
    ts.schedule.flushFile = 1;
    return;
  }

  // Logging
  INFO("Flushing timestamp data");

  // Flush actual file
  if (FATFS_sync(ts.fp) <= 0)
    WARN("Failed to sync filesystem!");

  DBUG("done...");
  // Clear schedule
  ts.schedule.flushFile = 0;
}

void TIME_yield ()
{
  // Process a metastamp
  if (!_fff_is_empty((ts.fifo.metastamp))) {
    const metastamp_t meta = _fff_read((ts.fifo.metastamp));
    INFO("Processing metastamp (%s)", meta.comment);
    if(f_printf(ts.fp, "%lu,,,%s" FATFS_EOL, meta.sample, meta.comment) <= 0 ){
      WARN("failed write");
    }

    // Only run one task per yield loop
    return;
  }
  // Ensure there is nothing in FIFO before syncing
  else if (ts.schedule.flushFile) return TIME_flush(1);
}
