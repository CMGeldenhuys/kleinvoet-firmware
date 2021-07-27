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
//#define WAVE_MOCK_WRITES

// Run Speedtests at startup
//#define FATFS_RUN_SPEEDTEST

// LED_ORANGE Timer setting for main file
#define LED_ORANGE_TIM &htim3
#define LED_ORANGE_TIM_CH TIM_CHANNEL_1

#ifdef __cplusplus
}
#endif

#endif //FIRMWARE_CONFIG_H
