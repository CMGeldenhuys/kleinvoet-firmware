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

extern RTC_HandleTypeDef hrtc;

static LOG_t log = {0};

static const char *LOG_Lvl_str_[] = {
        "DBUG",
        "INFO",
        "WARN",
        "ERR",
};

#ifdef LOG_COLOR
static const char *LOG_color_code_[] = {
        LOG_COLOR_FG_DEFAULT,
        LOG_COLOR_FG_BLUE,
        LOG_COLOR_FG_YELLOW,
        LOG_COLOR_FG_RED
};
#endif

static int LOG_timestamp_ (const char *funcName, LOG_Lvl_e lvl, char *buf);

int LOG_init()
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
  log.locked = 1;
  INFO("LOG_msg_t: %lu", sizeof(LOG_msg_t));
  INFO("LOG_t: %lu", sizeof(LOG_t));
  log.locked  = 0;
  INFO("Work buffer size: %lu", LOG_MSG_INFO_LEN + LOG_MSG_LEN + sizeof(LOG_EOL));

  return 1;
}

int LOG_log (const char *funcName, LOG_Lvl_e lvl, char *fmt, ...)
{
  // prevent logging before log device is ready (FILE OR TTY)
  if(!log.ready || log.locked) {
    log.missed++;
    return  -1;
  }

  if(log.missed > 0) {
    const uint_least16_t missed = log.missed;
    log.missed = 0;
    WARN("%d missed log entries before device was ready", missed);
  }

  log.locked = 1;

  // Create working buffer
  char workBuf[LOG_MSG_INFO_LEN + LOG_MSG_LEN + sizeof(LOG_EOL)];
  int  infoLen = LOG_timestamp_(funcName, lvl, workBuf);

  if (infoLen <= 0) return infoLen;

  va_list args;
  va_start(args, fmt);

  // Clamp infoLen to max
  if (infoLen > LOG_MSG_INFO_LEN) infoLen = LOG_MSG_INFO_LEN;

  int msgLen = vsnprintf(workBuf + infoLen, LOG_MSG_LEN, fmt, args);

  if (msgLen <= 0) {
    va_end(args);
    log.locked = 0;
    return msgLen;
  }

  // Clamp msgLen to max
  if (msgLen > LOG_MSG_LEN) msgLen = LOG_MSG_LEN;

  // Compute message length (faster than using strlen)
  size_t len = msgLen + infoLen;

  strcat(workBuf + len, LOG_EOL);
  len += sizeof(LOG_EOL);

  // Persist Log entry
  // -1 : remove NULL term from string
  size_t tmp;
  if ((tmp = strlen(workBuf)) != len - 1) {
    ERR("Fokop %u", tmp);
  }
  int bytesWritten = LOG_write((uint8_t *) workBuf, len - 1);

  if (bytesWritten > 0 && lvl == LOG_ERR) {
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