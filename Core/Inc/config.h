//
// Created by devsploit on 2020/09/14.
//

#ifndef FIRMWARE_CONFIG_H
#define FIRMWARE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_MAX_DELAY 100

#define AUTHORS "CM Geldenhuys"

#define VERSION_MAJOR "0"
#define VERSION_MINOR "6"
#define VERSION_PATCH "4"
#define VERSION_RC "alpha"

#if defined(VERSION_RC)
#define VERSION VERSION_MAJOR "." VERSION_MINOR "." VERSION_PATCH "-" VERSION_RC
#else
#define VERSION VERSION_MAJOR "." VERSION_MINOR "." VERSION_PATCH
#endif

// Set logging level
#define LOG_LEVEL_INFO
// Set Log destination
#define LOG_DEST_TTY

#ifdef DEBUG
#define PERF_ENABLE

// Change ADC sampling Freq.
#define ADC_SAMPLING_RATE 16000
#endif

// Enable static file allocation
#define WAVE_STATIC_FILE_ALLOC

// Enable mock writing of WAVE file
#define WAVE_MOCK_WRITES

// Run Speedtests at startup
//#define FATFS_RUN_SPEEDTEST

#ifdef __cplusplus
}
#endif

#endif //FIRMWARE_CONFIG_H
