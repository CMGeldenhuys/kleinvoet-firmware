/**
 *
 * @file logger.c
 * @author CM Geldenhuys
 * @date 6 Aug. 2020
 *
 * @headerfile logger.h "logger.h"
 *
 */

#include "logger.h"

#ifdef LOG_DEST_TTY

#include "tty.h"

#endif

extern RTC_HandleTypeDef hrtc;

static LOG_t log = {
        .ready = 0,
        .locked = 0,
        .missed = 0,
        .writer = {
                .flush = LOG_flush,
                .write = LOG_write,
        },
        .fifo = {0, 0, 0, {}},
        .workBuf = {}
};


static const char *LOG_Lvl_str_[] = {
        "DBUG",
        "INFO",
        "PERF",
        "WARN",
        "ERR",
};

#ifdef LOG_COLOR
static const char *LOG_color_code_[] = {
        LOG_COLOR_FG_DEFAULT,
        LOG_COLOR_FG_BLUE,
        LOG_COLOR_FG_GREEN,
        LOG_COLOR_FG_YELLOW,
        LOG_COLOR_FG_RED
};
#endif

static int LOG_timestamp_ (const char *funcName, LOG_Lvl_e lvl, char *buf);

int LOG_init ()
{
  log.ready = 1;

#if defined(LOG_LEVEL_ERROR)
  ERR("Logger running at level 'ERR'");
#elif defined(LOG_LEVEL_WARN)
  WARN("Logger running at level 'WARN'");
#elif defined(LOG_LEVEL_INFO)
  INFO("Logger running at level 'INFO'");
#elif defined(LOG_LEVEL_DEBUG)
  DBUG("Logger running at level 'DEBUG'");
#else
  DBUG("Logger running at unknown level");
#endif
  INFO("LOG_msg_t: %lu", sizeof(LOG_msg_t));
  INFO("LOG_t: %lu", sizeof(LOG_t));
  INFO("Work buffer size: %lu", LOG_MSG_INFO_LEN + LOG_MSG_LEN + sizeof(LOG_EOL));

  return 1;
}

int LOG_log (const char *funcName, LOG_Lvl_e lvl, char *fmt, ...)
{
  // prevent logging before log device is ready (FILE OR TTY)
  if (!log.ready || log.locked) {
#ifdef LOG_DEST_TTY
    TTY_printf("Missed log entry from %s %d %s" TTY_EOL, funcName, lvl, fmt);
#endif
    log.missed++;
    return -1;
  }

  if (log.missed > 0) {
    const uint_least16_t missed = log.missed;
    log.missed = 0;
    WARN("%d missed log entries before device was ready", missed);
  }

  // Lock log to prevent race conditions
  log.locked = 1;

  // Get timestamping and format
  // Store length of info string
  int infoLen = LOG_timestamp_(funcName, lvl, log.workBuf);

  // Failed to create timestamp
  if (infoLen <= 0) {
    log.locked = 0;
    return infoLen;
  }
    // Clamp infoLen to max
  else if (infoLen > LOG_MSG_INFO_LEN) infoLen = LOG_MSG_INFO_LEN;

  // Parse varargs
  va_list args;
  va_start(args, fmt);

  // Parse message format
  int msgLen = vsnprintf(log.workBuf + infoLen, LOG_MSG_LEN, fmt, args);

  // If failed to parse message
  if (msgLen <= 0) {
    // free vargs to prevent memory leak
    va_end(args);

    log.locked = 0;
    return msgLen;
  }
    // Clamp msgLen to max
  else if (msgLen > LOG_MSG_LEN) msgLen = LOG_MSG_LEN;

  // Compute message length (faster than using strlen)
  size_t len = msgLen + infoLen;

  // Add EOL to workBuf
  strcat(log.workBuf + len, LOG_EOL);
  len += sizeof(LOG_EOL);

  // Remove NULL terminator from length of string
  len--;

#ifdef DEBUG
  // Sanity check that compares computed strlen with  actual
  // -1 : remove NULL term from string
  size_t trueLen = strlen(log.workBuf);
  if (trueLen != len) {
    // TODO: add logger fifo so that logging in logging is possible
    ERR("String lengths do not match %u != %u", trueLen, len);
    len = trueLen;
  }
#endif

  // Persist Log entry
  int bytesWritten = log.writer.write((uint8_t *) log.workBuf, len);

  if (lvl == LOG_ERR && bytesWritten > 0) {
    // On Err force Log cache to be written
    // Write out Log cache/buffer
    LOG_flush();
  }

  log.locked = 0;
  va_end(args);
  return bytesWritten;
}

static int LOG_timestamp_ (const char *funcName, LOG_Lvl_e lvl, char *buf)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
#ifdef LOG_COLOR
  return snprintf(buf, LOG_MSG_INFO_LEN,
                  "%s[%4s:%-16.16s] %02x-%02x-%02x %02x:%02x:%02x.%-3lu %s- ",
                  LOG_color_code_[lvl],
                  LOG_Lvl_str_[lvl], funcName,
                  sDate.Year, sDate.Month, sDate.Date,
                  sTime.Hours, sTime.Minutes, sTime.Seconds, sTime.SubSeconds,
                  LOG_COLOR_RESET
  );
#else
  return snprintf(buf, LOG_MSG_INFO_LEN,
                     "[%4s:%-16.16s] %02x-%02x-%02x %02x:%02x:%02x.%-3lu - ",
                     LOG_Lvl_str_[lvl], funcName,
                     sDate.Year, sDate.Month, sDate.Date,
                     sTime.Hours, sTime.Minutes, sTime.Seconds, sTime.SubSeconds
                 );
#endif
}

__WEAK int LOG_write (__unused uint8_t *buf, __unused size_t len)
{
  return -1;
}

__WEAK int LOG_flush ()
{
  return -1;
}