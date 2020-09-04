/**
 *
 * @file gps.h
 * @author CM Geldenhuys
 * @date 31 Aug. 2020
 *
 * @brief NMEA GPS parsing library.
 *
 * [Link](https://bit.ly/34NOscf) to NMEA protocol for U-Blox M8 module.
 *
 */

#ifndef GPS_H_
#define GPS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "serial.h"

#ifndef GPS_BUF_LEN
#define GPS_BUF_LEN 32
#endif

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
} GPS_UBX_Class_e;

typedef enum __attribute__ ((packed)) {
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
} NMEA_Msg_id_e;

typedef struct {
    GPS_UBX_Class_e cls;
    uint8_t         id;
    uint16_t        len;
    uint8_t         payload[];
} GPS_UBX_cmd_t;

typedef struct {
    uint8_t             sync_1;
    uint8_t             sync_2;
    const GPS_UBX_cmd_t *cmd;
    uint8_t             CK_A;
    uint8_t             CK_B;
} GPS_UBX_msg_t;

typedef enum {
    GPS_IDLE,

    GPS_RX_SYNC_2,
    GPS_RX_PREAMBLE,
    GPS_RX_PAYLOAD,
    GPS_RX_CK_A,
    GPS_RX_CK_B,
    GPS_RX_DONE
} GPS_state_e;

typedef struct {
    Serial_t serial;
    struct {
        union {
            uint8_t mem[GPS_BUF_LEN];
            GPS_UBX_cmd_t _t;
        } cmd;
        size_t idx;
        uint8_t CK_A;
        uint8_t CK_B;

        GPS_state_e state;
    } rx;
} GPS_t;

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_UBX_Class_e cls;
        uint8_t         id;
        uint16_t        len;

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
        GPS_UBX_Class_e cls;
        uint8_t         id;
        uint16_t        len;

        uint8_t msgClass;
        uint8_t msgID;
        uint8_t rate;
    };
} UBX_CFG_MSG_t;

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_UBX_Class_e cls;
        uint8_t         id;
        uint16_t        len;

        uint32_t        iTOW;
        int32_t          clkB;
        int32_t          clkD;
        uint32_t        tAcc;
        uint32_t        fAcc;
    };
} UBX_NAV_CLK_t;

// Magic Numbers
#define GPS_SYNC_1_ 0xB5
#define GPS_SYNC_2_ 0x62
#define GPS_PREAMBLE_LEN_ sizeof(GPS_UBX_cmd_t)

#define UBX_PORT_DDC    (0u)
#define UBX_PORT_UART1  (1u)
#define UBX_PORT_USB    (3u)
#define UBX_PORT_SPI    (4u)

#define UBX_CFG_PRT_TXREADY_DISABLE     (0x0000)

#define UBX_CFG_PRT_MODE_CHARLEN_5      (0b00 << 6u)
#define UBX_CFG_PRT_MODE_CHARLEN_6      (0b01 << 6u)
#define UBX_CFG_PRT_MODE_CHARLEN_7      (0b10 << 6u)
#define UBX_CFG_PRT_MODE_CHARLEN_8      (0b11 << 6u)

#define UBX_CFG_PRT_MODE_PARTIY_EVEN    (0b000 << 9u)
#define UBX_CFG_PRT_MODE_PARTIY_ODD     (0b001 << 9u)
#define UBX_CFG_PRT_MODE_PARTIY_NO      (0b100 << 9u)

#define UBX_CFG_PRT_MODE_NSTOPBITS_1_0  (0b00 << 12u)
#define UBX_CFG_PRT_MODE_NSTOPBITS_1_5  (0b01 << 12u)
#define UBX_CFG_PRT_MODE_NSTOPBITS_2_0  (0b10 << 12u)
#define UBX_CFG_PRT_MODE_NSTOPBITS_0_5  (0b11 << 12u)

#define UBX_CFG_PRT_PROTO_UBX           (0x0001)
#define UBX_CFG_PRT_PROTO_NMEA          (0x0002)

// UBX Commands
static const UBX_CFG_PRT_t GPS_DEFAULT_PORT_CONFIG = {
        .cls            = UBX_CFG,
        .id             = 0x00,
        .len            = 20,
        .portID         = UBX_PORT_UART1,
        .txReady        = UBX_CFG_PRT_TXREADY_DISABLE,
        .mode           = UBX_CFG_PRT_MODE_CHARLEN_8
                          | UBX_CFG_PRT_MODE_PARTIY_NO
                          | UBX_CFG_PRT_MODE_NSTOPBITS_1_0,
        .baudRate       = 115200ul,
        .inProtoMask    = UBX_CFG_PRT_PROTO_UBX,
        .outProtoMask   = UBX_CFG_PRT_PROTO_UBX
};

static const UBX_CFG_PRT_t GPS_GET_PORT_CONFIG = {
        .cls            = UBX_CFG,
        .id             = 0x00,
        .len            = 1,
        .portID         = UBX_PORT_UART1
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_DTM = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_DTM,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GBQ = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GBQ,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GBS = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GBS,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GGA = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GGA,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GLL = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GLL,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GLQ = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GLQ,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GNQ = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GNQ,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GNS = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GNS,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GPQ = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GPQ,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GRS = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GRS,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GSA = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GSA,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GST = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GST,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_GSV = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_GSV,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_RMC = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_RMC,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_TXT = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_TXT,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_VLW = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_VLW,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_VTG = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_VTG,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_DISABLE_NMEA_ZDA = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = NMEA_STD,
        .msgID    = NMEA_ZDA,
        .rate     = 0
};

static const UBX_CFG_MSG_t GPS_ENABLE_UBX_NAV_CLK = {
        .cls      = UBX_CFG,
        .id       = 0x01,
        .len      = 3,

        .msgClass = UBX_NAV,
        .msgID    = 0x22,
        .rate     = 5
};

#define GPS_LEN_DEFAULT_CONFIG (sizeof(GPS_DEFAULT_CONFIG)/sizeof(GPS_UBX_cmd_t))

static const GPS_UBX_cmd_t *const GPS_DEFAULT_CONFIG[] = {
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

        // Enable GPS time
        &GPS_ENABLE_UBX_NAV_CLK.generic
};


//static const GPS_UBX_cmd_t UBX_CFG_MSG = {
//        .class   = 0x06,
//        .ID      = 0x01,
//        .len     = 3,
//        .payload = {0x00, 0x00, 0x00}
//};

//static const GPS_UBX_cmd_t UBX_CFG_PRT = {
//        .class   = 0x06,
//        .ID      = 0x00,
//        .len     = 20,
//        .payload = {
//                0x00, // portID
//                0x00,  // reserved
//                0x0000, // txReady
//                0x00000000, // mode
//                9600ul,
//
//
//        }
//};


int GPS_init (UART_HandleTypeDef *uart);

int GPS_yield ();

#ifdef __cplusplus
}
#endif

#endif // GPS_H_
