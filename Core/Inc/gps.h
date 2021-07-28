/**
 *
 * @file gps.h
 * @author CM Geldenhuys
 * @date 31 Aug. 2020
 *
 * @brief UBX GPS parsing library for UBlox M8Q
 *
 *
 */

#ifndef GPS_H_
#define GPS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "serial.h"
#include "fatfs.h"

#include <assert.h>

#ifndef GPS_BUF_LEN
#define GPS_BUF_LEN 512
#endif

#define GPS_MAX_CMD_LEN ((GPS_BUF_LEN) - (GPS_PREAMBLE_LEN_))
//                              cls                 id                len
#define UBX_HEADER_SIZE (sizeof(GPS_cls_e) + sizeof(uint8_t) + sizeof(uint16_t))

#define UBX_SIZEOF_PAYLOAD(__packet__) (sizeof(__packet__) - UBX_HEADER_SIZE)

#define bitfield_t uint8_t
#define UBX_BIT_ENABLE (1U)
#define UBX_BIT_DISBALE (0U)

// Magic Numbers
#define GPS_UBX_SYNC_BYTE_1 0xB5
#define GPS_UBX_SYNC_BYTE_2 0x62
#define GPS_NMEA_ID '$'
#define GPS_NMEA_END_1 '\r'
#define GPS_NMEA_END_2 '\n'
#define GPS_PREAMBLE_LEN_ sizeof(GPS_UBX_cmd_t)

typedef enum __attribute__ ((packed)) {
    UBX_NAV = 0x01,
    UBX_RXM = 0x02,
    UBX_INF = 0x04,
    UBX_ACK = 0x05,
    UBX_CFG = 0x06,
    UBX_UPD = 0x09,
    UBX_MON = 0x0A,
    UBX_AID = 0x0B,
    UBX_TIM = 0x0D,
    UBX_ESF = 0x10,
    UBX_MGA = 0x13,
    UBX_LOG = 0x21,
    UBX_SEC = 0x27,
    UBX_HNR = 0x28,

    NMEA_STD = 0xF0,
    NMEA_PBX = 0xF1
} GPS_cls_e;

#define UBX_ACK_ACK  0x01
#define UBX_ACK_NACK 0x00

typedef enum __attribute__ ((packed)) {
    UBX_NAV_CLK       = 0x22,
    UBX_NAV_TIMEUTC   = 0x21,
    UBX_NAV_STATUS    = 0x03,
    UBX_NAV_SAT       = 0x35,
    UBX_NAV_HPPOSECEF = 0x13,
    UBX_NAV_POSECEF   = 0x01,
    UBX_NAV_PVT       = 0x07,
    UBX_CFG_PRT       = 0x00,
    UBX_CFG_MSG       = 0x01,
    UBX_CFG_TP5       = 0x31,
    UBX_CFG_NAV5      = 0x24,

    NMEA_DTM = 0x0A,
    NMEA_GBQ = 0x44,
    NMEA_GBS = 0x09,
    NMEA_GGA = 0x00,
    NMEA_GLL = 0x01,
    NMEA_GLQ = 0x43,
    NMEA_GNQ = 0x42,
    NMEA_GNS = 0x0D,
    NMEA_GPQ = 0x40,
    NMEA_GRS = 0x06,
    NMEA_GSA = 0x02,
    NMEA_GST = 0x07,
    NMEA_GSV = 0x03,
    NMEA_RMC = 0x04,
    NMEA_TXT = 0x41,
    NMEA_VLW = 0x0F,
    NMEA_VTG = 0x05,
    NMEA_ZDA = 0x08

} GPS_id_e;

typedef struct {
    GPS_cls_e cls;
    GPS_id_e  id;
    uint16_t  len;
    uint8_t   payload[];
} GPS_UBX_cmd_t;

typedef struct {
    uint8_t             sync_1;
    uint8_t             sync_2;
    const GPS_UBX_cmd_t *cmd;
    uint8_t             CK_A;
    uint8_t             CK_B;
} GPS_UBX_msg_t;

typedef enum {
    GPS_UNDEF  = 0x000000,
    GPS_IDLE   = 0x010000,
    GPS_CONFIG = 0x800000,
    GPS_RX     = 0x040000,
} GPS_state_e;

#define GPS_RX_OK_STATUS_CODE(__OP__) GPS_RX_STATUS_CODE((unsigned)(__OP__) & 0x7FU)
#define GPS_RX_ERR_STAUS_CODE(__OP__) GPS_RX_STATUS_CODE((unsigned)(__OP__) | 0x80U)
#define GPS_RX_STATUS_CODE(__OP__) (((unsigned)(__OP__) & 0xFFU) << 24U )
#define GPS_RX_STATE(__STATE__) ((unsigned)(__STATE__) & 0xFFU)
#define _NEXT_STATE(__STATE__) (GPS_RX_STATE(__STATE__) << 8U)
#define GPS_RX_ADVANCE_STATE(__STATE__) (((unsigned)(__STATE__) & 0xFF00U) >> 8U)

// "Sub-state" Status Codes:
// Success Codes
#define GPS_RX_OK    GPS_RX_OK_0
#define GPS_RX_OK_0  GPS_RX_OK_STATUS_CODE(0)
#define GPS_RX_OK_1  GPS_RX_OK_STATUS_CODE(1)
#define GPS_RX_OK_2  GPS_RX_OK_STATUS_CODE(2)
#define GPS_RX_OK_3  GPS_RX_OK_STATUS_CODE(3)
// Error Codes
#define GPS_RX_ERR_1  GPS_RX_ERR_STAUS_CODE(-1)
#define GPS_RX_ERR_2  GPS_RX_ERR_STAUS_CODE(-2)
#define GPS_RX_ERR_3  GPS_RX_ERR_STAUS_CODE(-3)

typedef enum {
    // Current State  : 0b....|....|....|....|....|....|XXXX|XXXX
    // Next State     : 0b....|....|....|....|XXXX|XXXX|....|....
    // ERROR          : 0b1XXX|XXXX|....|....|....|....|....|....
    // OK             : 0b0XXX|XXXX|....|....|....|....|....|....

    // States:
    GPS_RX_RESET                        = GPS_RX_STATE(0),
    GPS_RX_WAIT                         = GPS_RX_STATE(1),
    GPS_RX_POTENTIAL_COMMAND            = GPS_RX_STATE(2),
    GPS_RX_PREAMBLE                     = GPS_RX_STATE(3),
    GPS_RX_PAYLOAD                      = GPS_RX_STATE(4),
    GPS_RX_CK_A                         = GPS_RX_STATE(5),
    GPS_RX_CK_B                         = GPS_RX_STATE(6),
    GPS_RX_CHECKSUM                     = GPS_RX_STATE(7),
    GPS_RX_NMEA                         = GPS_RX_STATE(8),
    GPS_RX_NMEA_END                     = GPS_RX_STATE(9),

    // Unrelated states
    GPS_RX_NO_DATA                      = GPS_RX_STATE(-2),
    GPS_RX_UNKNOWN                      = GPS_RX_STATE(-1),

    // State Transitions:                 STATUS        | NEXT STATE                            | CURRENT STATE
    // GPS_RX_WAIT                        ---------------------------------------------------------------------------------
    GPS_RX_WAITING                      = GPS_RX_OK_0   | _NEXT_STATE(GPS_RX_WAIT)              | GPS_RX_WAIT,
    GPS_RX_UBX_DET                      = GPS_RX_OK_1   | _NEXT_STATE(GPS_RX_POTENTIAL_COMMAND) | GPS_RX_WAIT,
    GPS_RX_NMEA_DET                     = GPS_RX_OK_2   | _NEXT_STATE(GPS_RX_NMEA)             | GPS_RX_WAIT,
    // GPS_RX_NMEA                        ---------------------------------------------------------------------------------
    GPS_RX_NMEA_PENDING                 = GPS_RX_OK_0   | _NEXT_STATE(GPS_RX_NMEA)              | GPS_RX_NMEA,
    GPS_RX_NMEA_CR                      = GPS_RX_OK_1   | _NEXT_STATE(GPS_RX_NMEA_END)          | GPS_RX_NMEA,
    GPS_RX_NMEA_UBX_DET                 = GPS_RX_ERR_1  | _NEXT_STATE(GPS_RX_RESET)             | GPS_RX_NMEA,
    GPS_RX_NMEA_OVERFLOW                = GPS_RX_ERR_2  | _NEXT_STATE(GPS_RX_RESET)             | GPS_RX_NMEA,
    // GPS_RX_NMEA                        ---------------------------------------------------------------------------------
    GPS_RX_NMEA_END_SUCESS              = GPS_RX_OK_0   | _NEXT_STATE(GPS_RX_RESET)             | GPS_RX_NMEA_END,
    GPS_RX_NMEA_END_FAIL                = GPS_RX_ERR_1  | _NEXT_STATE(GPS_RX_NMEA)              | GPS_RX_NMEA_END,
    // GPS_RX_POTENTIAL_COMMAND           ---------------------------------------------------------------------------------
    GPS_RX_POTENTIAL_COMMAND_OK         = GPS_RX_OK_0   | _NEXT_STATE(GPS_RX_PREAMBLE)          | GPS_RX_POTENTIAL_COMMAND,
    GPS_RX_POTENTIAL_COMMAND_UNEXPECTED = GPS_RX_ERR_1  | _NEXT_STATE(GPS_RX_RESET)             | GPS_RX_POTENTIAL_COMMAND,
    // GPS_RX_PREAMBLE                    ---------------------------------------------------------------------------------
    GPS_RX_PREAMBLE_COMPLETE            = GPS_RX_OK_0   | _NEXT_STATE(GPS_RX_PAYLOAD)           | GPS_RX_PREAMBLE,
    GPS_RX_PREAMBLE_PENDING             = GPS_RX_OK_1   | _NEXT_STATE(GPS_RX_PREAMBLE)          | GPS_RX_PREAMBLE,
    GPS_RX_PREAMBLE_OVERFLOW            = GPS_RX_ERR_1  | _NEXT_STATE(GPS_RX_RESET)             | GPS_RX_PREAMBLE,
    // GPS_RX_PAYLOAD                     ---------------------------------------------------------------------------------
    GPS_RX_PAYLOAD_OK                   = GPS_RX_OK_0   | _NEXT_STATE(GPS_RX_CK_A)              | GPS_RX_PAYLOAD,
    GPS_RX_PAYLOAD_PENDING              = GPS_RX_OK_1   | _NEXT_STATE(GPS_RX_PAYLOAD)           | GPS_RX_PAYLOAD,
    GPS_RX_PAYLOAD_OVERFLOW             = GPS_RX_ERR_1  | _NEXT_STATE(GPS_RX_RESET)             | GPS_RX_PAYLOAD,
    // GPS_RX_CK_A                        ---------------------------------------------------------------------------------
    GPS_RX_CK_A_ACK                     = GPS_RX_OK_0   | _NEXT_STATE(GPS_RX_CK_B)              | GPS_RX_CK_A,
    // GPS_RX_CHECKSUM                    ---------------------------------------------------------------------------------
    GPS_RX_CHECKSUM_PASS                = GPS_RX_OK_0   | _NEXT_STATE(GPS_RX_RESET)             | GPS_RX_CHECKSUM,
    GPS_RX_CHECKSUM_FAIL                = GPS_RX_ERR_1  | _NEXT_STATE(GPS_RX_RESET)             | GPS_RX_CHECKSUM,

} GPS_rx_state_e;

typedef struct {
    size_t  idx;
    uint8_t CK_A;
    uint8_t CK_B;
    union {
        uint8_t       mem[GPS_BUF_LEN];
        char          nmea[GPS_BUF_LEN];
        GPS_UBX_cmd_t ubx;
    }       cmd;
} GPS_rx_cmd_buffer_t;
static_assert(GPS_BUF_LEN > GPS_PREAMBLE_LEN_, "GPS_BUF_LEN must be greater than preamble length ");

typedef struct {
    Serial_t serial;
    GPS_rx_cmd_buffer_t rxBuff;
    GPS_rx_state_e rxState;

    GPS_state_e state;

    uint8_t timeValid;
    uint_least16_t timeValidN;
    uint_least16_t timeInvalidN;
} GPS_t;

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint8_t  portID;
        uint8_t  _reserved1;
        union {
            struct {
                // LSB
                bitfield_t en: 1;
                bitfield_t pol: 1;
                bitfield_t pin: 5;
                uint16_t thres: 9;
                // MSB
            };
            uint16_t bitmask;
        } txReady;

        union {
            struct {
                // LSB
                bitfield_t _reserved1: 6;
                enum __packed {
                    UBX_CFG_PRT_MODE_CHAR_LEN_5 = 0U,
                    UBX_CFG_PRT_MODE_CHAR_LEN_6 = 1U,
                    UBX_CFG_PRT_MODE_CHAR_LEN_7 = 2U,
                    UBX_CFG_PRT_MODE_CHAR_LEN_8 = 3U,
                } charLen : 2;
                bitfield_t _reserved2: 1;
                enum __packed {
                    UBX_CFG_PRT_MODE_PARITY_EVEN = 0b000,
                    UBX_CFG_PRT_MODE_PARITY_ODD = 0b001,
                    UBX_CFG_PRT_MODE_PARITY_NO = 0b100,
                } parity : 3;
                enum __packed {
                    UBX_CFG_PRT_MODE_STOP_BITS_1_0 = 0U,
                    UBX_CFG_PRT_MODE_STOP_BITS_1_5 = 1U,
                    UBX_CFG_PRT_MODE_STOP_BITS_2_0 = 2U,
                    UBX_CFG_PRT_MODE_STOP_BITS_0_5 = 3U,
                } nStopBits : 2;
                bitfield_t _reserved3: 2;
                bitfield_t _reserved4: 8;
                bitfield_t _reserved5: 8;
                // MSB
            };
            uint32_t bitmask;
        } mode;

        uint32_t baudRate;
        union {
            struct {
                // LSB
                bitfield_t Ubx: 1;
                bitfield_t Nmea: 1;
                bitfield_t Rtcm: 1;
                bitfield_t _reserved1: 2;
                bitfield_t Rctm3: 1;
                bitfield_t _reserved2: 2;
                bitfield_t _reserved3: 8;
                // MSB
            };
            uint16_t bitmask;
        } inProtoMask;

        union {
            struct {
                // LSB
                bitfield_t Ubx: 1;
                bitfield_t Nmea: 1;
                bitfield_t _reserved1: 3;
                bitfield_t Rctm3: 1;
                bitfield_t _reserved2: 2;
                bitfield_t _reserved3: 8;
                // MSB
            };
            uint16_t bitmask;
        } outProtoMask;

        union {
            struct {
                // LSB
                bitfield_t _reserved1: 1;
                bitfield_t extendedTxTimeout: 1;
                bitfield_t _reserved2: 6;
                bitfield_t _reserved3: 8;
                // MSB
            };
            uint16_t bitmask;
        } flags;

        uint8_t  _reserved2[2];
    };
} UBX_CFG_PRT_t;
#define UBX_CFG_PRT_PAYLOAD_SIZE 20
static_assert(UBX_SIZEOF_PAYLOAD(UBX_CFG_PRT_t) == UBX_CFG_PRT_PAYLOAD_SIZE, "UBX_CFG_PRT_t payload size mismatch");

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint8_t  tpIdx;
        uint8_t  version;
        uint8_t  reserved1[2];
        int16_t  antCableDelay;
        int16_t  rfGroupDelay;
        uint32_t freqPeriod;
        uint32_t freqPeriodLock;
        uint32_t pulseLenRatio;
        uint32_t pulseLenRatioLock;
        int32_t  userConfigDelay;
        uint32_t flags;
    };
} UBX_CFG_TP5_t;

// New Style of message structuring
// TODO migrate all messages to this style
// TODO Add docs to each field
typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        union {
            struct {
                // LSB
                bitfield_t dyn: 1;
                bitfield_t minEl: 1;
                bitfield_t posFixMode: 1;
                bitfield_t drLim: 1;
                bitfield_t posMask: 1;
                bitfield_t timeMask: 1;
                bitfield_t staticHoldMask: 1;
                bitfield_t dgpsMask: 1;
                bitfield_t cnoThreshold: 1;
                bitfield_t reserved_2: 1;
                bitfield_t utc: 1;
                bitfield_t reserved_1: 5;
                // MSB
            };
            uint16_t mask;
        };

        enum {
            UBX_CFG_NAV5_DYNMODEL_PORTABLE    = 0U,
            UBX_CFG_NAV5_DYNMODEL_STATIONARY  = 2U,
            UBX_CFG_NAV5_DYNMODEL_PEDESTRIAN  = 3U,
            UBX_CFG_NAV5_DYNMODEL_AUTOMOTIVE  = 4U,
            UBX_CFG_NAV5_DYNMODEL_SEA         = 5U,
            UBX_CFG_NAV5_DYNMODEL_AIRBORNE_1G = 6U,
            UBX_CFG_NAV5_DYNMODEL_AIRBORNE_2G = 7U,
            UBX_CFG_NAV5_DYNMODEL_AIRBORNE_4G = 8U,
            UBX_CFG_NAV5_DYNMODEL_WRIST       = 9U,
            UBX_CFG_NAV5_DYNMODEL_BIKE        = 10U
        }         dynModel: 8;

        enum {
            UBX_CFG_NAV5_FIXMODE_2D   = 1U,
            UBX_CFG_NAV5_FIXMODE_3D   = 2U,
            UBX_CFG_NAV5_FIXMODE_AUTO = 3U
        }         fixMode: 8;

#define UBX_CFG_NAV5_FIXED_ALT_CONV_RATE (0.01f)
#define UBX_CFG_NAV5_FIXED_ALT_CONV(__alt__) ((__alt__) / UBX_CFG_NAV5_FIXED_ALT_CONV_RATE)
        int32_t fixedAlt; // m

#define UBX_CFG_NAV5_FIXED_ALT_VAR_CONV_RATE (0.0001f)
#define UBX_CFG_NAV5_FIXED_ALT_VAR_CONV(__altVar__) ((__altVar__) / UBX_CFG_NAV5_FIXED_ALT_VAR_CONV_RATE)
        uint32_t fixedAltVar; // m^2

        int8_t minElev; // deg

        uint8_t reserved_drLimit; // s

#define UBX_CFG_NAV5_FIXED_DOP_CONV_RATE (0.1f)
#define UBX_CFG_NAV5_FIXED_DOP_CONV(__dop__) ((__dop__) / UBX_CFG_NAV5_FIXED_DOP_CONV_RATE)
        uint16_t pDop;
        uint16_t tDop;

        uint16_t pAcc;
        uint16_t tAcc;

        uint8_t staticHoldeThresh;

        uint8_t dgnssTimeout;

        uint8_t cnoThreshNumSVs;

        uint8_t cnoThresh;

        uint8_t reserved1[2];

        uint16_t staticHoldMaxDist;

        enum {
            UBX_CFG_NAV5_UTC_STANDARD_AUTO        = 0U,
            UBX_CFG_NAV5_UTC_STANDARD_UTC_USNO    = 3U,
            UBX_CFG_NAV5_UTC_STANDARD_UTC_GALILEO = 5U,
            UBX_CFG_NAV5_UTC_STANDARD_UTC_GLONASS = 6U,
            UBX_CFG_NAV5_UTC_STANDARD_UTC_BEIDOU  = 7U
        } utcStandard: 8;

        uint8_t reserved2[2];
    };
} UBX_CFG_NAV5_t;
#define UBX_CFG_NAV5_PAYLOAD_SIZE 36
static_assert(UBX_SIZEOF_PAYLOAD(UBX_CFG_NAV5_t) == UBX_CFG_NAV5_PAYLOAD_SIZE, "UBX_CFG_NAV5_t payload size mismatch");

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint8_t msgClass;
        uint8_t msgID;
        uint8_t rate;
    };
} UBX_CFG_MSG_t;

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        GPS_cls_e msgClsID;
        uint8_t   msgID;
    };
} UBX_ACK_t;

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint32_t iTOW;
        int32_t  clkB;
        int32_t  clkD;
        uint32_t tAcc;
        uint32_t fAcc;
    };
} UBX_NAV_CLK_t;

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint32_t iTOW;
        uint32_t tAcc;
        int32_t  nano;
        uint16_t year;
        uint8_t  month;
        uint8_t  day;
        uint8_t  hour;
        uint8_t  min;
        uint8_t  sec;

        union {
            struct {
                // LSB
                bitfield_t validTOW: 1;
                bitfield_t validWKN: 1;
                bitfield_t validUTC: 1;
                bitfield_t _reserved1: 1;
                __packed enum {
                    UBX_NAV_TIMEUTC_STANDARD_NO_INFORMATION = 0U,
                    UBX_NAV_TIMEUTC_STANDARD_CRL            = 1U,
                    UBX_NAV_TIMEUTC_STANDARD_NIST           = 2U,
                    UBX_NAV_TIMEUTC_STANDARD_USNO           = 3U,
                    UBX_NAV_TIMEUTC_STANDARD_BIPM           = 4U,
                    UBX_NAV_TIMEUTC_STANDARD_EU             = 5U,
                    UBX_NAV_TIMEUTC_STANDARD_SU             = 6U,
                    UBX_NAV_TIMEUTC_STANDARD_NTSC           = 7U,
                    UBX_NAV_TIMEUTC_STANDARD_UNKNOWN        = 15U
                }          utcStandard: 4;
                // MSB
            };
            uint8_t valid;
        };

    };
} UBX_NAV_TIMEUTC_t;
#define UBX_NAV_TIMEUTC_PAYLOAD_SIZE 20
static_assert(UBX_SIZEOF_PAYLOAD(UBX_NAV_TIMEUTC_t) == UBX_NAV_TIMEUTC_PAYLOAD_SIZE, "UBX_CFG_NAV5_t payload size mismatch");

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint32_t iTOW;
        enum {
            UBX_NAV_STATUS_GPSFIX_NOFIX             = 0x00U,
            UBX_NAV_STATUS_GPSFIX_DEADRECKONING     = 0x01U,
            UBX_NAV_STATUS_GPSFIX_2D                = 0x02U,
            UBX_NAV_STATUS_GPSFIX_3D                = 0x03U,
            UBX_NAV_STATUS_GPSFIX_GPS_DEADRECKONING = 0x04U,
            UBX_NAV_STATUS_GPSFIX_TIME              = 0x05U
        }        gpsFix: 8;
        union {
            struct {
                // LSB
                bitfield_t gpsFixOk: 1;
                bitfield_t diffSoln: 1;
                bitfield_t wknSet: 1;
                bitfield_t towSet: 1;
                bitfield_t _reserved1: 4;
                // MSB
            };
            uint8_t flags;
        };
        union {
            struct {
                // LSB
                bitfield_t diffCorr: 1;
                bitfield_t carrSolnValid: 1;
                bitfield_t _reserved2: 4;
                __packed enum {
                    UBX_NAV_STATUS_MAPMATCHING_NONE                = 0b00,
                    UBX_NAV_STATUS_MAPMATCHING_VALID_EXPIRED       = 0b01,
                    UBX_NAV_STATUS_MAPMATCHING_VALID               = 0b10,
                    UBX_NAV_STATUS_MAPMATCHING_VALID_DEADRECKONING = 0b11
                }          mapMatching: 2;
                // MSB
            };
            uint8_t fixStat;
        };
        union {
            struct {
                // LSB
                __packed enum {
                    UBX_NAV_STATUS_PSMSTATE_ACQUISTITION             = 0,
                    UBX_NAV_STATUS_PSMSTATE_TRACKING                 = 1,
                    UBX_NAV_STATUS_PSMSTATE_POWER_OPTIMISED_TRACKING = 2,
                    UBX_NAV_STATUS_PSMSTATE_INACTIVE                 = 3
                } psmState: 2;

                bitfield_t _reserved3: 1;

                __packed enum {
                    UBX_NAV_STATUS_SPOOFDETSTATE_UNKNOWN            = 0,
                    UBX_NAV_STATUS_SPOOFDETSTATE_NO_INDICATION      = 1,
                    UBX_NAV_STATUS_SPOOFDETSTATE_INDICATION         = 2,
                    UBX_NAV_STATUS_SPOOFDETSTATE_MULTIPLE_LOCATIONS = 3
                }          spoofDetState: 2;

                bitfield_t _reserved4: 1;

                __packed enum {
                    UBX_NAV_STATUS_CARRSOLN_NO_CARRIER           = 0,
                    UBX_NAV_STATUS_CARRSOLN_FLOATING_AMBIGUITIES = 1,
                    UBX_NAV_STATUS_CARRSOLN_FIXED_AMBIGUITIES    = 2
                }          carrSoln: 2;
                // MSB
            };
            uint8_t flags2;
        };
        uint32_t ttff;
        uint32_t msss;
    };
} UBX_NAV_STATUS_t;
#define UBX_NAV_STATUS_PAYLOAD_SIZE 16
static_assert(UBX_SIZEOF_PAYLOAD(UBX_NAV_STATUS_t) == UBX_NAV_STATUS_PAYLOAD_SIZE, "UBX_CFG_NAV5_t payload size mismatch");

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint32_t iTOW;
        uint8_t  version;
        uint8_t  numSvs;
        uint8_t  reserved[2];

        struct {
            uint8_t  gnssId;
            uint8_t  svId;
            uint8_t  cno;
            int8_t   elev;
            int16_t  azim;
            int16_t  prRes;
            uint32_t flags;
        }        svs[];
    };
} UBX_NAV_SAT_t;

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint8_t  version;
        uint8_t  reserved1[3];
        uint32_t iTOW;
        int32_t  ecefX;
        int32_t  ecefY;
        int32_t  ecefZ;
        int8_t   ecefXHp;
        int8_t   ecefYHp;
        int8_t   ecefZHp;
        union {
            struct {
                // LSB
                bitfield_t invalidEcef: 1;
                bitfield_t padding1: 7;
                // MSB
            };
            uint8_t flags;
        };
        uint32_t pAcc;
    };
} UBX_NAV_HPPOSECEF_t;
#define UBX_NAV_HPPOSECEF_PAYLOAD_SIZE 28
static_assert(UBX_SIZEOF_PAYLOAD(UBX_NAV_HPPOSECEF_t) == UBX_NAV_HPPOSECEF_PAYLOAD_SIZE,
              "UBX_NAV_HPPOSECEF_t payload size mismatch");

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint32_t iTOW;
        int32_t  ecefX;
        int32_t  ecefY;
        int32_t  ecefZ;
        uint32_t pAcc;
    };
} UBX_NAV_POSECEF_t;
#define UBX_NAV_POSECEF_PAYLOAD_SIZE 20
static_assert(UBX_SIZEOF_PAYLOAD(UBX_NAV_POSECEF_t) == UBX_NAV_POSECEF_PAYLOAD_SIZE,
              "UBX_NAV_POSECEF_t payload size mismatch");

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint32_t iTOW;

        uint16_t year;
        uint8_t month;
        uint8_t day;

        uint8_t hour;
        uint8_t min;
        uint8_t sec;

        union {
            struct {
                // LSB
                bitfield_t validDate : 1;
                bitfield_t validTime : 1;
                bitfield_t fullyResolved: 1;
                bitfield_t validMag : 1;
                bitfield_t _reserved2 : 4;
                // MSB
            };
            uint8_t valid;
        };

        uint32_t tAcc;
        int32_t nano;

        enum {
            UBX_NAV_PVT_FIX_TYPE_NO_FIX = 0U,
            UBX_NAV_PVT_FIX_TYPE_DEAD_RECKONING = 1U,
            UBX_NAV_PVT_FIX_TYPE_2D = 2U,
            UBX_NAV_PVT_FIX_TYPE_3D = 3U,
            UBX_NAV_PVT_FIX_TYPE_COMBINED = 4U,
            UBX_NAV_PVT_FIX_TYPE_TIME = 5U

        } fixType : 8;

        union {
            struct {
                // LSB
                bitfield_t gnssFixOK : 1;
                bitfield_t diffSoln : 1;
                bitfield_t _reserved3 : 3;
                bitfield_t headVehValid : 1;
                enum __packed {
                    UBX_NAV_PVT_FLAGS2_CARR_SOLN_NO_CARRIER = 0U,
                    UBX_NAV_PVT_FLAGS2_CARR_SOLN_FLOATING_AMBIGUITIES = 1U,
                    UBX_NAV_PVT_FLAGS2_CARR_SOLN_FIXED_AMBIGUITITES = 2U
                } carrSoln : 2;
                // MSB
            };
            uint8_t flags;
        };

        union {
          struct {
              // LSB
              bitfield_t _reserved4 : 5;
              bitfield_t confirmedAvai : 1;
              bitfield_t confirmedDate : 1;
              bitfield_t confirmedTime : 1;
              // MSB
          };

          uint8_t flags2;
        };


        uint8_t numSV;

        int32_t lon;
        int32_t lat;

        int32_t height;
        int32_t hMSL;
        uint32_t hAcc;
        uint32_t vAcc;

        int32_t velN;
        int32_t velE;
        int32_t velD;
        int32_t gSpeed;
        int32_t headMot;
        uint32_t sAcc;
        uint32_t headAcc;

        uint16_t pDOP;

        union {
            struct {
                // LSB
                bitfield_t invalidLlh : 1;
                bitfield_t _reserved5 : 7;
                // MSB
            };
            uint8_t flags3;
        };

        uint8_t _reserved1[5];

        int32_t headVeh;

        int16_t magDec;
        int16_t magAcc;

    };
} UBX_NAV_PVT_t;
#define UBX_NAV_PVT_PAYLOAD_SIZE 92
static_assert(UBX_SIZEOF_PAYLOAD(UBX_NAV_PVT_t) == UBX_NAV_PVT_PAYLOAD_SIZE,
              "UBX_NAV_PVT_t payload size mismatch");

#define UBX_PORT_DDC    (0U)
#define UBX_PORT_UART1  (1U)
#define UBX_PORT_USB    (3U)
#define UBX_PORT_SPI    (4U)

#define UBX_NAV_TIMEUTC_VALIDUTC        (0b00000100U)

#define UBX_CFG_TP5_FLAGS_ACTIVE          (0b00000001U)
#define UBX_CFG_TP5_FLAGS_LOCK_GPS_FREQ   (0b00000010U)
#define UBX_CFG_TP5_FLAGS_LOCK_OTHER_SET  (0b00000100U)
#define UBX_CFG_TP5_FLAGS_IS_FREQ         (0b00001000U)
#define UBX_CFG_TP5_FLAGS_IS_LENGTH       (0b00010000U)
#define UBX_CFG_TP5_FLAGS_ALIGN_TO_TOW    (0b00100000U)
#define UBX_CFG_TP5_FLAGS_POLARITY_RISING (0b01000000U)
#define UBX_CFG_TP5_FLAGS_GRID_UTC_GPS    (0b10000000U)
#define UBX_CFG_TP5_TIMEPULSE             (0U)
#define UBX_CFG_TP5_TIMEPULSE2            (1U)

// UBX Commands
static const UBX_CFG_PRT_t GPS_DEFAULT_PORT_CONFIG = {
        .cls            = UBX_CFG,
        .id             = UBX_CFG_PRT,
        .len            = 20u,
        .portID         = UBX_PORT_UART1,
        .txReady        = {.en = 0},
        .mode           = {
                .charLen  = UBX_CFG_PRT_MODE_CHAR_LEN_8,
                .parity   = UBX_CFG_PRT_MODE_PARITY_NO,
                .nStopBits= UBX_CFG_PRT_MODE_STOP_BITS_1_0
        },
        .baudRate       = 9600UL,
        .inProtoMask    = {.Ubx = 1},
        .outProtoMask   = {.Ubx = 1},
};

static const UBX_CFG_PRT_t GPS_GET_PORT_CONFIG = {
        .cls            = UBX_CFG,
        .id             = UBX_CFG_PRT,
        .len            = 1u,
        .portID         = UBX_PORT_UART1
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_DTM = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_DTM,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GBQ = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GBQ,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GBS = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GBS,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GGA = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GGA,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GLL = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GLL,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GLQ = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GLQ,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GNQ = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GNQ,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GNS = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GNS,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GPQ = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GPQ,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GRS = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GRS,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GSA = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GSA,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GST = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GST,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GSV = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GSV,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_RMC = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_RMC,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_TXT = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_TXT,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_VLW = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_VLW,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_VTG = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_VTG,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_ZDA = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_ZDA,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_ENABLE_UBX_NAV_CLK = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = UBX_NAV,
        .msgID    = UBX_NAV_CLK,
        .rate     = 10u
};

static const UBX_CFG_MSG_t GPS_ENABLE_UBX_NAV_STATUS = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = UBX_NAV,
        .msgID    = UBX_NAV_STATUS,
        .rate     = 10u
};

static const UBX_CFG_MSG_t GPS_ENABLE_UBX_NAV_SAT = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = UBX_NAV,
        .msgID    = UBX_NAV_SAT,
        .rate     = 10u
};

static const UBX_CFG_MSG_t GPS_ENABLE_UBX_NAV_TIMEUTC = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = UBX_NAV,
        .msgID    = UBX_NAV_TIMEUTC,
        .rate     = 1u
};

static const UBX_CFG_MSG_t GPS_ENABLE_UBX_NAV_POSECEF = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = UBX_NAV,
        .msgID    = UBX_NAV_POSECEF,
        .rate     = 60u
};

static const UBX_CFG_MSG_t GPS_DISABLE_UBX_NAV_SAT = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = UBX_NAV,
        .msgID    = UBX_NAV_SAT,
        .rate     = 0u
};

static const UBX_CFG_MSG_t GPS_ENABLE_UBX_NAV_PVT = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = UBX_NAV,
        .msgID    = UBX_NAV_PVT,
        .rate     = 60u
};

static const UBX_CFG_TP5_t GPS_CONFIGURE_TIMEPULSE = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_TP5,
        .len      = 32u,

        .tpIdx              = UBX_CFG_TP5_TIMEPULSE,
        .antCableDelay      = 50, // ns
        .freqPeriod         = 0, // Hz
        .freqPeriodLock     = 1,
        .pulseLenRatio      = 0, // us
        .pulseLenRatioLock  = 10000,
        .userConfigDelay    = 0, // ns
        .flags              = UBX_CFG_TP5_FLAGS_ACTIVE
                              | UBX_CFG_TP5_FLAGS_LOCK_GPS_FREQ
                              | UBX_CFG_TP5_FLAGS_LOCK_OTHER_SET
                              | UBX_CFG_TP5_FLAGS_IS_FREQ
                              | UBX_CFG_TP5_FLAGS_IS_LENGTH
                              | UBX_CFG_TP5_FLAGS_ALIGN_TO_TOW
                              | UBX_CFG_TP5_FLAGS_POLARITY_RISING
};

static const UBX_CFG_NAV5_t GPS_CONFIGURE_NAV5 = {
        .cls = UBX_CFG,
        .id = UBX_CFG_NAV5,
        .len = UBX_CFG_NAV5_PAYLOAD_SIZE,

        .dyn = UBX_BIT_ENABLE,
        .dynModel = UBX_CFG_NAV5_DYNMODEL_STATIONARY,

        .staticHoldMask = UBX_BIT_ENABLE,
        .staticHoldMaxDist = 2, // m
        .staticHoldeThresh = 100, // cm/s
};

#define GPS_LEN_DEFAULT_CONFIG (sizeof(GPS_DEFAULT_CONFIG)/sizeof(GPS_UBX_cmd_t))

static const GPS_UBX_cmd_t *const GPS_DEFAULT_CONFIG[] = {
        // Set default port config
        &GPS_DEFAULT_PORT_CONFIG.generic,

        // Disable all NMEA messages
        &GPS_DISABLE_NMEA_DTM.generic,
        &GPS_DISABLE_NMEA_GBQ.generic,
        &GPS_DISABLE_NMEA_GBS.generic,
        &GPS_DISABLE_NMEA_GGA.generic,
        &GPS_DISABLE_NMEA_GLL.generic,
        &GPS_DISABLE_NMEA_GLQ.generic,
        &GPS_DISABLE_NMEA_GNQ.generic,
        &GPS_DISABLE_NMEA_GNS.generic,
        &GPS_DISABLE_NMEA_GPQ.generic,
        &GPS_DISABLE_NMEA_GRS.generic,
        &GPS_DISABLE_NMEA_GSA.generic,
        &GPS_DISABLE_NMEA_GST.generic,
        &GPS_DISABLE_NMEA_GSV.generic,
        &GPS_DISABLE_NMEA_RMC.generic,
        &GPS_DISABLE_NMEA_TXT.generic,
        &GPS_DISABLE_NMEA_VLW.generic,
        &GPS_DISABLE_NMEA_VTG.generic,
        &GPS_DISABLE_NMEA_ZDA.generic,

        // Enable GPS messages
        &GPS_ENABLE_UBX_NAV_SAT.generic,
        &GPS_ENABLE_UBX_NAV_STATUS.generic,
        &GPS_ENABLE_UBX_NAV_TIMEUTC.generic,
        &GPS_ENABLE_UBX_NAV_POSECEF.generic,
//        &GPS_ENABLE_UBX_NAV_PVT.generic,

        // Setup time pulse
        &GPS_CONFIGURE_TIMEPULSE.generic,

        // Setup NAV
        &GPS_CONFIGURE_NAV5.generic
};

/**
 * @brief Initialise GPS module
 *
 * @param [in] uart Communications serial port used
 *
 * @return Returns a value greater than  0 on success
 */
int GPS_init (UART_HandleTypeDef *uart);

/**
 * @brief Run state machine for current loop
 *
 * @return Returns value greater than 0 on success
 */
int GPS_yield ();

/**
 * @brief Send command to GPS over serial
 *
 * @param [in] txCmd Message to send
 * @param [in] waitAck Block and wait till message ACK is received
 * @param [in] retryOnNack If command failed retry
 *
 * @return Returns value greater than 0 on success
 *
 * @note waitAck is not implemented
 * @note retryOnNack is not implemented
 */
int GPS_sendCommand (const GPS_UBX_cmd_t *txCmd, int waitAck, int retryOnNack);

#define GPS_log_UBX_NAV_TIMEUTC(cmd_t) \
({ \
    INFO("UBX-NAV-TIMEUTC (%c)", \
         cmd_t->valid & UBX_NAV_TIMEUTC_VALIDUTC \
         ? '1' \
         : '0');                \
                                      \
    DBUG("  iTOW: %lu", cmd_t->iTOW); \
    DBUG("  tAcc: %lu", cmd_t->tAcc); \
    DBUG("  nano: %0l", cmd_t->nano); \
    DBUG("  year: %u", cmd_t->year); \
    DBUG("  month: %02u", cmd_t->month); \
    DBUG("  day: %02u", cmd_t->day); \
    DBUG("  hour: %02u", cmd_t->hour); \
    DBUG("  min: %02u", cmd_t->min); \
    DBUG("  sec: %02u", cmd_t->sec); \
    DBUG("  validTOW: %u", cmd_t->validTOW);     \
    DBUG("  validWKN: %u", cmd_t->validWKN);        \
    DBUG("  validUTC: %u", cmd_t->validUTC); \
    DBUG("  utcStandard: %u", cmd_t->utcStandard); \
})

#define GPS_log_UBX_NAV_STATUS(cmd_t) \
({                                    \
  INFO("UBX-NAV-STATUS (V:%d, N:%d)", gps.timeValidN, gps.timeInvalidN); \
  DBUG("  iTOW: %lu", cmd_t->iTOW); \
  DBUG("  gpsFix: 0x%02X", cmd_t->gpsFix);                               \
  DBUG("  gpsFixOk: 0x%02X", cmd_t->gpsFixOk); \
  DBUG("  diffSoln: 0x%02X", cmd_t->diffSoln); \
  DBUG("  wknSet: 0x%02X", cmd_t->wknSet); \
  DBUG("  towSet: 0x%02X", cmd_t->towSet); \
  DBUG("  diffCorr: 0x%02X", cmd_t->diffCorr); \
  DBUG("  carrSolnValid: 0x%02X", cmd_t->carrSolnValid); \
  DBUG("  mapMatching: 0x%02X", cmd_t->mapMatching); \
  DBUG("  psmState: 0x%02X", cmd_t->psmState); \
  DBUG("  spoofDetState: 0x%02X", cmd_t->spoofDetState); \
  DBUG("  carrSoln: 0x%02X", cmd_t->carrSoln); \
  DBUG("  ttff: %lu", cmd_t->ttff); \
  DBUG("  msss: %lu", cmd_t->msss); \
})

#define GPS_log_UBX_NAV_SAT(cmd_t) \
({                                 \
  INFO("UBX-NAV-SAT (%u)", cmd_t->numSvs); \
  \
  DBUG("  iTOW: %lu", cmd_t->iTOW); \
  DBUG("  version: %u", cmd_t->version); \
  DBUG("  numSvs: %u", cmd_t->numSvs); \
  \
  for (uint8_t i = 0; i < cmd_t->numSvs; i++) { \
    DBUG("  ---- Svs %u ----", i); \
    DBUG("    gnssId: %u", cmd_t->svs[i].gnssId); \
    DBUG("    svId: %u", cmd_t->svs[i].svId); \
    DBUG("    cno: %u", cmd_t->svs[i].cno); \
    DBUG("    elev: %d", cmd_t->svs[i].elev); \
    DBUG("    azim: %d", cmd_t->svs[i].azim); \
    DBUG("    prRes: %d", cmd_t->svs[i].prRes); \
    DBUG("    flags: 0x%02X", cmd_t->svs[i].flags); \
  } \
})

#define GPS_log_UBX_NAV_CLK(cmd_t) \
({                                 \
  INFO("UBX-NAV-CLK"); \
  DBUG("  iTOW: %lu", cmd_t->iTOW); \
  DBUG("  clkB: %l", cmd_t->clkB); \
  DBUG("  clkD: %l", cmd_t->clkD); \
  DBUG("  tAcc: %lu", cmd_t->tAcc); \
  DBUG("  fAcc: %lu", cmd_t->fAcc); \
})

#define GPS_log_UBX_NAV_HPPOSECEF(cmd_t) \
({                                 \
  INFO("UBX-NAV-HPPOSECEF (%l, %l, %l)", cmd_t->ecefX, cmd_t->ecefY, cmd_t->ecefZ); \
  DBUG("  version: %u", cmd_t->version);      \
  DBUG("  iTOW: %lu", cmd_t->iTOW);      \
  DBUG("  ecefX: %l", cmd_t->ecefX);     \
  DBUG("  ecefY: %l", cmd_t->ecefY);      \
  DBUG("  ecefZ: %l", cmd_t->ecefZ);      \
  DBUG("  ecefXHp: %d", cmd_t->ecefXHp);      \
  DBUG("  ecefYHp: %d", cmd_t->ecefYHp);      \
  DBUG("  ecefZHp: %d", cmd_t->ecefZHp);      \
  DBUG("  flags.invalidEcef: %u", cmd_t->invalidEcef);      \
  DBUG("  pAcc: %lu", cmd_t->pAcc);      \
})

#define GPS_log_UBX_NAV_POSECEF(cmd_t) \
({                                 \
  INFO("UBX-NAV-POSECEF (%d, %d, %d, %u)", cmd_t->ecefX, cmd_t->ecefY, cmd_t->ecefZ, cmd_t->pAcc); \
  DBUG("  iTOW: %u", cmd_t->iTOW);      \
  DBUG("  ecefX: %d", cmd_t->ecefX);     \
  DBUG("  ecefY: %d", cmd_t->ecefY);      \
  DBUG("  ecefZ: %d", cmd_t->ecefZ);      \
  DBUG("  pAcc: %u", cmd_t->pAcc);      \
})

#define GPS_log_UBX_NAV_PVT(cmd_t) \
({                                 \
  INFO("UBX-NAV-PVT");               \
  DBUG("  iTOW: %d", cmd_t->iTOW);    \
  INFO("  year: %u", cmd_t->year);    \
  INFO("  month: %u", cmd_t->month);    \
  INFO("  day: %u", cmd_t->day);    \
  INFO("  hour: %u", cmd_t->hour);    \
  INFO("  min: %u", cmd_t->min);    \
  INFO("  sec: %u", cmd_t->sec);    \
  INFO("  nano: %d", cmd_t->nano);    \
  INFO("  tAcc: %u", cmd_t->tAcc);    \
  INFO("  fixType: %u", cmd_t->fixType);    \
  INFO("  validDate: %u", cmd_t->validDate);    \
  INFO("  validTime: %u", cmd_t->validTime);    \
  INFO("  fullyResolved: %u", cmd_t->fullyResolved);    \
  DBUG("  validMag: %u", cmd_t->validMag);    \
  INFO("  gnssFixOK: %u", cmd_t->gnssFixOK);    \
  DBUG("  diffSoln: %u", cmd_t->diffSoln);    \
  DBUG("  headVehValid: %u", cmd_t->headVehValid);    \
  DBUG("  carrSoln: %u", cmd_t->carrSoln);    \
  INFO("  numSV: %u", cmd_t->numSV);    \
  INFO("  lon: %d", cmd_t->lon);    \
  INFO("  lat: %d", cmd_t->lat);    \
  INFO("  height: %d", cmd_t->height);    \
  INFO("  hMSL: %d", cmd_t->hMSL);    \
  INFO("  hAcc: %u", cmd_t->hAcc);    \
  INFO("  vAcc: %u", cmd_t->vAcc);    \
  DBUG("  velN: %d", cmd_t->velN);    \
  DBUG("  velE: %d", cmd_t->velE);    \
  DBUG("  velD: %d", cmd_t->velD);    \
  DBUG("  gSpeed: %d", cmd_t->gSpeed);    \
  DBUG("  headMot: %d", cmd_t->headMot);    \
  DBUG("  sAcc: %d", cmd_t->sAcc);    \
  DBUG("  headAcc: %d", cmd_t->headAcc);    \
  DBUG("  pDOP: %u", cmd_t->pDOP);    \
  DBUG("  invalidLlh: %u", cmd_t->invalidLlh);    \
  DBUG("  headVeh: %d", cmd_t->headVeh);    \
  DBUG("  magDec: %d", cmd_t->magDec);    \
  DBUG("  magAcc: %u", cmd_t->magAcc);    \
})

#ifdef __cplusplus
}
#endif

#endif // GPS_H_
