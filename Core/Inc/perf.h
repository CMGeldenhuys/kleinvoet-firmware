//
// Created by CM Geldenhuys on 2021/02/24.
//

#ifndef FIRMWARE_PERF_H
#define FIRMWARE_PERF_H

#include "config.h"

#ifndef PERF_LOG
#define PERF_LOG(_name_, _msg_, ...) LOG_log(_name_, LOG_PERF, (_msg_), ##__VA_ARGS__)
#endif

#define PERF_THRESHOLD(_threshold_) (threshold = _threshold_)

#ifdef PERF_ENABLE
#define PERF_START(_name_) { \
  uint32_t threshold = 0; \
  const uint32_t tick = HAL_GetTick();

#define PERF_END(_name_)  \
  const uint32_t tock = HAL_GetTick(); \
  const uint32_t runTime = tock - tick; \
  const int32_t delta = runTime - threshold; \
  if(delta > 0) { \
    PERF_LOG(_name_, "Ran for %lu (%d)", runTime, delta); \
  } \
}

#else

#define PERF_START(_name_)

#define PERF_END(_name_)

#endif

#endif //FIRMWARE_PERF_H
