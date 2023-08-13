//
// Created by CM GELDENHUYS <CMGELDENHUYS AT SUN DOT AC DOT ZA> on 2023/08/13.
//

#ifndef KLEINVOET_TELEMETRY_H
#define KLEINVOET_TELEMETRY_H

#include "stddef.h"
#include "inttypes.h"


/**
 * @brief Telemetry function return status.
 *
 * All fail states are zero or negative, thus checking if the return value is greater than zero, is equivalent to
 * checking if the returned telemetry function has succeeded.
 *
 * @code{C}
 *  tel_ret_status status = tel_...()
 *  if status > 0 {
 *      // SUCCESS
 *  } else if status == TEL_NOT_CONFIGURED {
 *      // HANDLE NOT CONFIGURED
 *  } else {
 *      // FAILED
 *  }
 */
typedef enum {
    /** Telemetry module not initialised yet. */
    TEL_NOT_INITIALIZED = -3,
    /** Called telemetry function is not configured. */
    TEL_NOT_CONFIGURED = -2,
    /** Called telemetry function is configured, but is not yet able to return a value. */
    TEL_NOT_READY = -1,
    /** General telemetry failed response. */
    TEL_FAILED = 0,
    /** Returned value is OK and valid. */
    TEL_OK = 1
} tel_ret_status;

/**
 * @brief Initialize telemetry module.
 *
 * @return Status code.
 */
tel_ret_status tel_init();

/**
 * @brief Battery voltage in millivolts.
 *
 * Telemetry function to get battery voltage in millivolts.
 *
 * @note Does not force ADC conversion, returns cached sample.
 *
 * @param[out] batt_mv Return telemetry battery voltage.
 *
 * @return Status code.
*/
tel_ret_status tel_get_battery_voltage_mv(int16_t *batt_mv);

#define TEL_DEV_EVENT_MASK_ERR      0x80000000
#define TEL_DEV_EVENT_MASK_WARN     0x40000000
#define TEL_DEV_EVENT_MASK_INFO     0x20000000
#define TEL_DEV_EVENT_MASK_OTHER    0x10000000
/**
 * @brief Enum containing current running state of device. Used to report device events
 */
typedef enum {
    // ERROR STATES
    /** Error event, device has halted operation. */
    ERR_DEVICE_HALTED = 0x00000001 | TEL_DEV_EVENT_MASK_ERR,
    // WARNING STATES
    /** Warning event, low battery has been detected. */
    WARN_LOW_BATT     = 0x00000001 | TEL_DEV_EVENT_MASK_WARN,
    /** Warning event, low storage has been detected. */
    WARN_LOW_STORAGE  = 0x00000002 | TEL_DEV_EVENT_MASK_WARN,
    // INFO STATES
    /** Informative event, audio has clipped. */
    INFO_AUDIO_CLIP   = 0x00000001 | TEL_DEV_EVENT_MASK_INFO,
    // OTHER STATES
    NO_EVENT          = 0x00000000 | TEL_DEV_EVENT_MASK_OTHER,
} tel_device_event_e;

/**
 * @brief Device event
 *
 * Telemetry function reporting any note worthy event that has occurred on the device.
 *
 * @note This function is not idempotent, repeated calls may result in side effects (when `flag_clear_event` is set).
 *
 * @param[out] dev_event Return device telemetry event.
 *
 * @param flag_clear_event Should the returned event be cleared from queue.
 *
 * @return Status code.
 */
tel_ret_status tel_get_device_event(tel_device_event_e *dev_event, uint8_t flag_clear_event);

/**
 * Struct containing GNSS reported position in ECEF format.
 */
typedef struct {
    int32_t ecef_x_cm;
    int32_t ecef_y_cm;
    int32_t ecef_z_cm;
    uint32_t acc_cm;
} tel_gnss_pos_t;

/**
 * @brief Device's current GNSS position in ECEF format.
 *
 * Telemetry function for retrieving device's current GNSS position in ECEF format.
 *
 * @note Units are in centimetres (cm).
 *
 * @param[out] gnss_pos Returned GNSS position.
 *
 * @return Status code.
 */
tel_ret_status tel_get_gnss_pos(tel_gnss_pos_t *gnss_pos);

/**
 * @brief Number of GNSS satellites tracked.
 *
 * Telemetry function for retrieving number of actively tracked GNSS satellites.
 *
 * @param[out] gnss_nsat Returned number of GNSS satellites tracked.
 *
 * @return Status code.
 */
tel_ret_status tel_get_gnss_nsat(uint16_t *gnss_nsat);

#define TEL_GNSS_STAT_TIME_LOCKED 0x00000001
#define TEL_GNSS_STAT_TIME_VALID  0x00000002
#define TEL_GNSS_STAT_POS_LOCKED  0x00000010
#define TEL_GNSS_STAT_POS_VALID   0x00000020
/** Type alias for GNSS status. */
typedef uint32_t tel_gnss_stat_t;

/**
 * @brief GNSS module status.
 *
 * Telemetry function for retrieving GNSS current status.
 *
 * @param[out] gnss_stat Returned GNSS status.
 *
 * @return Status code.
 */
tel_ret_status tel_get_gnss_stat(tel_gnss_stat_t *gnss_stat);

/**
 * Packed struct containing timestamp data.
 *
 * @note __reserved field is used to align struct in memory (multiple of 8 bytes).
 */
typedef struct {
    uint32_t acc;
    int32_t nano;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t __reserved; // required for memory alignment
} tel_gnss_timestamp_t;

/**
 * @brief GNSS Timestamp.
 *
 * Telemetry function for retrieving GNSS Timestamp information.
 *
 * @param[out] timestamp Returned GNSS timestamp.
 *
 * @return Status code.
 */
tel_ret_status tel_get_gnss_timestamp(tel_gnss_timestamp_t *timestamp);

/**
 * @brief Available storage on SD card.
 *
 * Telemetry function for retrieving amount of free space available on SD card.
 *
 * @note Returned units are in KiB (assuming 512 byte sectors).
 *
 * @note The available space is computed for the mounted partition.
 *
 * @param[out] avail_kib Returned free space on SD card in KiB.
 *
 * @return Status code.
 */
tel_ret_status tel_get_storage_avail_kib(uint32_t *avail_kib);

/**
 * @brief Storage space used on SD card.
 *
 * Telemetry function for retrieving amount of space occupied on SD card.
 *
 * @note Returned units are in KiB (assuming 512 byte sectors)
 *
 * @note The used space is computed for the mounted partition.
 *
 * @param[out] used_kib Return used space on SD card in KiB.
 *
 * @return Status code.
 */
tel_ret_status tel_get_storage_used_kib(uint32_t *used_kib);

/**
 * @brief Recording filename.
 *
 * Telemetry function for retrieving current recording's base filename and path.
 *
 * @param[out] rec_filename Returned filename of recording.
 *
 * @param[in] buf_size Size of input buffer (rec_filename). See `strlcpy` for reference.
 *
 * @return Status code.
 */
tel_ret_status tel_get_rec_filename(char *rec_filename, size_t buf_size);

/**
 * @brief Recording duration.
 *
 * Telemetry function for retrieving current recording duration, in number of samples.
 * Requires configured sampling rate to compute time.
 *
 * @param[out] rec_dur_samples Return number of samples recorded.
 *
 * @return Status code.
 */
tel_ret_status tel_get_rec_duration(uint32_t *rec_dur_samples);

/**
 * @brief Recording available.
 *
 * Telemetry function for retrieving amount of samples that can theoretically be recorded to device.
 * Required configured sampling rate to compute time.
 *
 * @param[out] rec_avail_samples Return number of available samples.
 *
 * @return Status code.
 */
tel_ret_status tel_get_rec_avail(uint32_t *rec_avail_samples);

#define TEL_REC_CONF_STATE_MASK         0x03000000
#define TEL_REC_CONF_STATE_PAUSED       0x01000000
#define TEL_REC_CONF_STATE_REC          0x02000000
#define TEL_REC_CONF_SAMPLE_RATE_MASK   0x00FFFFFF
#define TEL_REC_CONF_NCHANNELS_MASK     0xF0000000
/** Type alias for recording configuration. */
typedef uint32_t tel_rec_config_t;

/**
 * @brief Recording configuration
 *
 * Telemetry function for retrieving recording configuration.
 *
 * @param[out] rec_config Return current recording configuration.
 *
 * @return Status code.
 */
tel_ret_status tel_get_rec_config(tel_rec_config_t *rec_config);

/**
 * @brief Log entry at line.
 *
 * Telemetry function for retrieving log entry at a given line in the logs.
 *
 * @param[in] line Line number to return.
 *
 * @param[out] log_line Returned log buffer.
 *
 * @param[in] buf_size Input buffer size. See `strlcpy` for reference.
 *
 * @return Status code.
 */
tel_ret_status tel_get_log_entry(uint32_t line, char *log_line, size_t buf_size);

/**
 * @brief Number of error log entries
 *
 * Telemetry function for retrieving number of log entries with log level of error.
 *
 * @note This metric is only valid if logs are configured to log errors (ERR).
 *
 * @param[out] num_errors Returned number of errors in log.
 *
 * @return Status code.
 */
tel_ret_status tel_get_log_num_error(uint16_t *num_errors);

/**
 * @brief Number of warning log entries
 *
 * Telemetry function for retrieving number of log entries with log level of warning.
 *
 * @note This metric is only valid if logs are configured to log warnings (WARN).
 *
 * @param[out] num_warns Returned number of errors in log.
 *
 * @return Status code.
 */
tel_ret_status tel_get_log_num_warn(uint16_t *num_warns);

#define TEL_DEV_CONF_ADC_MASK           0x0000000F
#define TEL_DEV_CONF_ADC_EN_MASK        0x00000001
#define TEL_DEV_CONF_LOG_MASK           0x000000F0
#define TEL_DEV_CONF_LOG_DEV_MASK       0x00000000
#define TEL_DEV_CONF_LOG_DEV_SD         0x00000000
#define TEL_DEV_CONF_LOG_DEV_TTY        0x00000000
#define TEL_DEV_CONF_LOG_LEVEL_MASK     0x00000030
#define TEL_DEV_CONF_LOG_LEVEL_INFO     0x00000010
#define TEL_DEV_CONF_LOG_LEVEL_WARN     0x00000020
#define TEL_DEV_CONF_LOG_LEVEL_ERR      0x00000030
#define TEL_DEV_CONF_GNSS_MASK          0x00000F00
#define TEL_DEV_CONF_GNSS_EN_MASK       0x00000100
#define TEL_DEV_CONF_LORA_MASK          0x0000F000
#define TEL_DEV_CONF_LORA_EN            0x00001000
#define TEL_DEV_CONF_TEL_MASK           0x000F0000
#define TEL_DEV_CONF_TEL_EN_MASK        0x00030000
#define TEL_DEV_CONF_TEL_EN             0x00010000
#define TEL_DEV_CONF_TEL_EN_MOCK        0x00020000
typedef uint32_t tel_device_config_t;
/**
 * @brief Device static configuration.
 *
 * Telemetry function for retrieving device configuration.
 *
 * @param[out] device_config Return device configuration, set at start up.
 *
 * @return Status code.
 */
tel_ret_status tel_get_device_config(tel_device_config_t *device_config);

#define TEL_DEV_FIRM_MAJOR_MASK 0xF0000000
#define TEL_DEV_FIRM_MINOR_MASK 0x0FFF0000
#define TEL_DEV_FIRM_PATCH_MASK 0x0000FFF0
#define TEL_DEV_FIRM_RESERVED   0x0000000E
#define TEL_DEV_FIRM_DIRTY_MASK 0x00000001
/** Type alias for device firmware version. */
typedef uint32_t tel_device_firmware_version_t;

/**
 * @brief Device firmware version.
 *
 * Telemetry function for retrieving device firmware version.
 *
 * @param[out] firmware_version Return device firmware version.
 *
 * @return Status code.
 */
tel_ret_status tel_get_device_firmware_version(tel_device_firmware_version_t* firmware_version);

/**
 * @brief Device short UUID.
 *
 * Telemetry function for retrieving device 32-bit UUID.
 *
 * @note Actual UUID is 96-bits, shortened version may have higher chance of collisions.
 *
 * @param[out] dev_uuid Return device shortened UUID.
 *
 * @return Status code.
 */
tel_ret_status tel_get_device_short_uuid(uint32_t * dev_uuid);

#ifdef MOCK_TELEMETRY
#define MOCK_TELEMETRY_NULL_IMPL
#ifdef MOCK_TELEMETRY_NULL_IMPL

#include "string.h"

tel_ret_status tel_init()
{
    return TEL_OK;
}

tel_ret_status tel_get_battery_voltage_mv(int16_t *batt_mv)
{
    *batt_mv = 0;

    return TEL_OK;
}

tel_ret_status tel_get_device_event(tel_device_event_e *dev_event, uint8_t flag_clear_event)
{
    *dev_event = NO_EVENT;

    return TEL_OK;
}

tel_ret_status tel_get_gnss_pos(tel_gnss_pos_t *gnss_pos)
{
    memset(gnss_pos, 0, sizeof(tel_gnss_pos_t));

    return TEL_OK;
}

tel_ret_status tel_get_gnss_nsat(uint16_t *gnss_nsat)
{
    *gnss_nsat = 0;

    return TEL_OK;
}

tel_ret_status tel_get_gnss_stat(tel_gnss_stat_t *gnss_stat)
{
    *gnss_stat = 0;

    return TEL_OK;
}

tel_ret_status tel_get_gnss_timestamp(tel_gnss_timestamp_t *timestamp)
{
    memset(timestamp, 0, sizeof(tel_gnss_timestamp_t));

    return TEL_OK;
}

tel_ret_status tel_get_storage_avail_kib(uint32_t *avail_kib)
{
    *avail_kib = 0;

    return TEL_OK;
}

tel_ret_status tel_get_storage_used_kib(uint32_t *used_kib)
{
    *used_kib = 0;

    return TEL_OK;
}

tel_ret_status tel_get_rec_filename(char *rec_filename, size_t buf_size)
{
    char mock_filename[] = "MOCK_REC.WAV";
    strlcpy(rec_filename, mock_filename, buf_size);

    return TEL_OK;
}

tel_ret_status tel_get_rec_duration(uint32_t *rec_dur_samples)
{
    *rec_dur_samples = 0;

    return TEL_OK;
}

tel_ret_status tel_get_rec_avail(uint32_t *rec_avail_samples)
{
    *rec_avail_samples = 0;

    return TEL_OK;
}

tel_ret_status tel_get_rec_config(tel_rec_config_t *rec_config)
{
    *rec_config = 0;

    return TEL_OK;
}

tel_ret_status tel_get_log_entry(uint32_t line, char * log_line, size_t buf_size)
{
    char mock_filename[] = "LOG ENTRY:: Hello, World!";
    strlcpy(log_line, mock_filename, buf_size);

    return TEL_OK;
}

tel_ret_status tel_get_log_num_error(uint16_t * num_errors)
{
    *num_errors = 0;

    return TEL_OK;
}

tel_ret_status tel_get_log_num_warn(uint16_t *num_warns)
{
    *num_warns = 0;

    return TEL_OK;
}

tel_ret_status tel_get_device_config(tel_device_config_t *device_config)
{
    *device_config = 0;

    return TEL_OK;
}

tel_ret_status tel_get_device_firmware_version(tel_device_firmware_version_t* firmware_version)
{
    *firmware_version = 0;

    return TEL_OK;
}

tel_ret_status tel_get_device_short_uuid(uint32_t * dev_uuid)
{
    *dev_uuid = 0;

    return TEL_OK;
}

#endif // MOCK_TELEMETRY_NULL_IMPL
#endif // MOCK_TELEMETRY

#endif //KLEINVOET_TELEMETRY_H
