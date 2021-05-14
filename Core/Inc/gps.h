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
#define GPS_BUF_LEN 128
#endif
//                              cls                 id                len
#define UBX_HEADER_SIZE (sizeof(GPS_cls_e) + sizeof(uint8_t) + sizeof(uint16_t))

#define UBX_SIZEOF_PAYLOAD(__packet__) (sizeof(__packet__) - UBX_HEADER_SIZE)

#define bitfield_t uint8_t
#define UBX_BIT_ENABLE (1U)
#define UBX_BIT_DISBALE (0U)

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

typedef enum __attribute__ ((packed)) {
    UBX_NAV_CLK       = 0x22,
    UBX_NAV_TIMEUTC   = 0x21,
    UBX_NAV_STATUS    = 0x03,
    UBX_NAV_SAT       = 0x35,
    UBX_NAV_HPPOSECEF = 0x13,
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

    GPS_RX_SYNC_2      = 0x040001,
    GPS_RX_PREAMBLE    = 0x040002,
    GPS_RX_PAYLOAD     = 0x040004,
    GPS_RX_CK_A        = 0x040008,
    GPS_RX_CK_B        = 0x040010,
    GPS_RX_CHECKSUM    = 0x040020,
    GPS_RX_PROCESS_CMD = 0x040040
} GPS_state_e;

typedef struct {
    Serial_t serial;
    struct {
        union {
            uint8_t       mem[GPS_BUF_LEN];
            GPS_UBX_cmd_t _t;
        }       cmd;
        size_t  idx;
        uint8_t CK_A;
        uint8_t CK_B;

        GPS_state_e state;
    }        rx;

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
        uint8_t  reserved1;
        uint16_t txReady;
        uint32_t mode;
        uint32_t baudRate;
        uint16_t inProtoMask;
        uint16_t outProtoMask;
        uint16_t flags;
        uint8_t  reserved2[2];
    };
} UBX_CFG_PRT_t;

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
        uint8_t  valid;
    };
} UBX_NAV_TIMEUTC_t;

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_cls_e cls;
        uint8_t   id;
        uint16_t  len;

        uint32_t iTOW;
        uint8_t  gpsFix;
        uint8_t  flags;
        uint8_t  fixStat;
        uint8_t  flags2;
        uint32_t ttff;
        uint32_t msss;
    };
} UBX_NAV_STATUS_t;

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

// Magic Numbers
#define GPS_SYNC_1_ 0xB5
#define GPS_SYNC_2_ 0x62
#define GPS_PREAMBLE_LEN_ sizeof(GPS_UBX_cmd_t)

#define UBX_PORT_DDC    (0U)
#define UBX_PORT_UART1  (1U)
#define UBX_PORT_USB    (3U)
#define UBX_PORT_SPI    (4U)

#define UBX_CFG_PRT_TXREADY_DISABLE     (0x0000U)

#define UBX_CFG_PRT_MODE_CHARLEN_5      (0b00U << 6U)
#define UBX_CFG_PRT_MODE_CHARLEN_6      (0b01U << 6U)
#define UBX_CFG_PRT_MODE_CHARLEN_7      (0b10U << 6U)
#define UBX_CFG_PRT_MODE_CHARLEN_8      (0b11U << 6U)

#define UBX_CFG_PRT_MODE_PARTIY_EVEN    (0b000U << 9U)
#define UBX_CFG_PRT_MODE_PARTIY_ODD     (0b001U << 9U)
#define UBX_CFG_PRT_MODE_PARTIY_NO      (0b100U << 9U)

#define UBX_CFG_PRT_MODE_NSTOPBITS_1_0  (0b00U << 12U)
#define UBX_CFG_PRT_MODE_NSTOPBITS_1_5  (0b01U << 12U)
#define UBX_CFG_PRT_MODE_NSTOPBITS_2_0  (0b10U << 12U)
#define UBX_CFG_PRT_MODE_NSTOPBITS_0_5  (0b11U << 12U)

#define UBX_CFG_PRT_PROTO_UBX           (0x0001U)
#define UBX_CFG_PRT_PROTO_NMEA          (0x0002U)

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
        .txReady        = UBX_CFG_PRT_TXREADY_DISABLE,
        .mode           = UBX_CFG_PRT_MODE_CHARLEN_8
                          | UBX_CFG_PRT_MODE_PARTIY_NO
                          | UBX_CFG_PRT_MODE_NSTOPBITS_1_0,
        .baudRate       = 9600UL,
        .inProtoMask    = UBX_CFG_PRT_PROTO_UBX,
        .outProtoMask   = UBX_CFG_PRT_PROTO_UBX
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

static const UBX_CFG_MSG_t GPS_ENABLE_UBX_NAV_HPPOSECEF = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = UBX_NAV,
        .msgID    = UBX_NAV_HPPOSECEF,
        .rate     = 10u
};

static const UBX_CFG_MSG_t GPS_DISABLE_UBX_NAV_SAT = {
        .cls      = UBX_CFG,
        .id       = UBX_CFG_MSG,
        .len      = 3u,

        .msgClass = UBX_NAV,
        .msgID    = UBX_NAV_SAT,
        .rate     = 0u
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
        &GPS_ENABLE_UBX_NAV_HPPOSECEF.generic,

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
 * @param [in] cmd Message to send
 * @param [in] waitAck Block and wait till message ACK is received
 * @param [in] retryOnNack If command failed retry
 *
 * @return Returns value greater than 0 on success
 *
 * @note waitAck is not implemented
 * @note retryOnNack is not implemented
 */
int GPS_sendCommand (const GPS_UBX_cmd_t *cmd, int waitAck, int retryOnNack);

#define GPS_log_UBX_NAV_TIMEUTC(cmd_t) \
({ \
    DBUG("UBX-NAV-TIMEUTC (%s)", \
         cmd_t->valid & UBX_NAV_TIMEUTC_VALIDUTC \
         ? "VALID" \
         : "INVALID");                \
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
    DBUG("  valid: 0x%02X", cmd_t->valid); \
})

#define GPS_log_UBX_NAV_STATUS(cmd_t) \
({                                    \
  INFO("UBX-NAV-STATUS (V:%d, N:%d)", gps.timeValidN, gps.timeInvalidN); \
  DBUG("  iTOW: %lu", cmd_t->iTOW); \
  DBUG("  gpsFix: 0x%02X", cmd_t->gpsFix); \
  DBUG("  flags: 0x%02X", cmd_t->flags); \
  DBUG("  fixStat: 0x%02x", cmd_t->fixStat); \
  DBUG("  flags2: 0x%02X", cmd_t->flags2); \
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
  INFO("UBX-NAV-HPPOSECEF"); \
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

#ifdef __cplusplus
}
#endif

#endif // GPS_H_
