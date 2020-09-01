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

#ifndef GPS_CMD_LEN
#define GPS_CMD_LEN 256
#endif

typedef struct {
    Serial_t serial;
    struct {
        char buf[GPS_CMD_LEN];
        char *pos;
        int  started;
    }        cmd;
} GPS_t;

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
    UBX_HNR = 0x28
} GPS_UBX_Class_e;

typedef struct {
    GPS_UBX_Class_e class;
    uint8_t         ID;
    uint16_t        len;
    uint8_t         payload[];
} GPS_UBX_cmd_t;

typedef struct {
    uint8_t             sync_1;
    uint8_t             sync_2;
    const GPS_UBX_cmd_t *cmd;
    uint8_t             CK_A;
    uint8_t             CK_B;
} GPS_UBX_t;

typedef union {
    GPS_UBX_cmd_t generic;
    struct {
        GPS_UBX_Class_e class;
        uint8_t         ID;
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
} UBX_CFG_PRT;

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
static const UBX_CFG_PRT GPS_DEFAULT_PORT_CONFIG = {
        .class          = UBX_CFG,
        .ID             = 0x00,
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

static const UBX_CFG_PRT GPS_GET_PORT_CONFIG = {
        .class          = UBX_CFG,
        .ID             = 0x00,
        .len            = 1,
        .portID         = UBX_PORT_UART1
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
