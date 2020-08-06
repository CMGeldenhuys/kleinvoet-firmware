//
// Created by CM Geldenhuys on 2020/08/06.
//

#include "logger.h"

extern RTC_HandleTypeDef hrtc;

static const char * LOG_Lvl_str_[] = {
  "DBUG",
  "INFO",
  "WARN",
  "ERR",
};

int LOG_timestamp_(const char* funcName, LOG_Lvl_e lvl, char *buf);

int LOG_log(const char *funcName, LOG_Lvl_e lvl, char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);

  // Create working buffer
  char workBuf[LOG_MSG_INFO_LEN + LOG_MSG_LEN + sizeof(LOG_EOL)];
  int infoLen = LOG_timestamp_(funcName, lvl, workBuf);

  if (infoLen <= 0) return infoLen;

  // Clamp infoLen to max
  if (infoLen > LOG_MSG_INFO_LEN) infoLen = LOG_MSG_INFO_LEN;

  int msgLen = vsnprintf(workBuf+infoLen, LOG_MSG_LEN, fmt, args);

  if (msgLen <= 0) return msgLen;

  // Clamp msgLen to max
  if (msgLen > LOG_MSG_LEN) msgLen = LOG_MSG_LEN;

  strcat(workBuf+msgLen, LOG_EOL);
  // Remove NULL termination since now byte array
  size_t len = msgLen + infoLen + sizeof(LOG_EOL) - 1;
  // Persist Log entry
  int bytesWritten = LOG_write((uint8_t *) workBuf, len);
  if ( bytesWritten > 0) {
    if (lvl == LOG_ERR){
      // Write out Log cache/buffer
      LOG_flush();
    }

    return 1;
  }
  return -1;
}

int LOG_timestamp_(const char* funcName, LOG_Lvl_e lvl, char *buf)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  return snprintf(buf, LOG_MSG_INFO_LEN,
                     "[%4s:%-16.16s] %02d-%02d-%02d %02d:%02d:%02d - ",
                     LOG_Lvl_str_[lvl], funcName,
                     sDate.Year, sDate.Month, sDate.Date,
                     sTime.Hours, sTime.Minutes, sTime.Seconds
           );
}

__WEAK int LOG_write(__unused uint8_t *buf, __unused size_t len)
{
  return -1;
}

__WEAK int LOG_flush()
{
  return -1;
}