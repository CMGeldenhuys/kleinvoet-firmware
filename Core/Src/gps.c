/**
 *
 * @file gps.c
 * @author CM Geldenhuys
 * @date 31 Aug. 2020
 *
 * @brief NMEA GPS parsing library.
 *
 * [Link](https://bit.ly/34NOscf) to NMEA protocol for U-Blox M8 module.
 *
 */

#include "gps.h"
#include "logger.h"

static GPS_t gps = {0};

void GPS_packMsg_ (GPS_UBX_t *ubx);

int GPS_checksum_ (GPS_UBX_t *ubx);

size_t GPS_cmdLen_ (const GPS_UBX_cmd_t *cmd);

int GPS_sendCommand (const GPS_UBX_cmd_t *cmd);

int GPS_configureUBX_ ();

int GPS_init (UART_HandleTypeDef *uart) {
  Serial_wrap(&gps.serial, uart);
  GPS_configureUBX_();
  return 1;
}

int GPS_configureUBX_ () {
  DBUG("Size of DEFAULT_CONFIG: %u", GPS_LEN_DEFAULT_CONFIG);
  for (size_t i = 0; i < GPS_LEN_DEFAULT_CONFIG; i++) {
    GPS_sendCommand(GPS_DEFAULT_CONFIG[i]);
  }

  return 1;
}

int GPS_sendCommand (const GPS_UBX_cmd_t *cmd) {
  // TODO: Implement ACk & NACK to retry
  GPS_UBX_t msg = {0};
  msg.cmd = cmd;
  DBUG("Sending UBX command 0x%02x - 0x%02x (%u bytes)", cmd->cls, cmd->id, cmd->len);
  GPS_packMsg_(&msg);

  // Send sync header
  Serial_write(&gps.serial, (uint8_t *) &msg, 2);
  // Send command
  Serial_write(&gps.serial, (uint8_t *) cmd, GPS_cmdLen_(cmd));
  // Send checksum
  Serial_write(&gps.serial, (uint8_t *) &msg.CK_A, 2);
}

inline size_t GPS_cmdLen_ (const GPS_UBX_cmd_t *cmd) {
  return GPS_PREAMBLE_LEN_ + cmd->len;
}

typedef union {
    uint8_t mem[64];
    GPS_UBX_cmd_t t;
}GPS_UBX_t_mem;
GPS_UBX_t_mem buf = {0};

int GPS_yield () {
  static size_t idx = 0;
  static uint8_t sync_1, sync_2, ck_a, ck_b;
  while (Serial_available(&gps.serial) > 0) {
    uint8_t c = Serial_read(&gps.serial);
    DBUG("recv: 0x%02X", c);

    if(gps.rxState == 0 && c == GPS_SYNC_1_) gps.rxState = 1;

    switch(gps.rxState) {
      case 1: {
        DBUG("recv sync1");
        sync_1 = c;
        gps.rxState = 2;
        break;
      }

      case 2: {
        DBUG("recv sync2");
        sync_2 = c;
        gps.rxState = 3;
        break;
      }

      case 3: {
        DBUG("recv cmd byte");
        buf.mem[idx++] = c;
        if(idx - GPS_PREAMBLE_LEN_ > buf.t.len) gps.rxState = 4;
        break;
      }

      case 4: {
        DBUG("recv ck_a");
        ck_a = c;
        gps.rxState = 5;
        break;
      }

      case 5: {
        DBUG("recv ck_b");
        ck_b = c;
        gps.rxState = 0;
        break;
      }

      default:
        break;
    }
  }
}

void GPS_packMsg_ (GPS_UBX_t *ubx) {
  // Place synchronisation characters for bus speed
  ubx->sync_1 = GPS_SYNC_1_;
  ubx->sync_2 = GPS_SYNC_2_;

  GPS_checksum_(ubx);
}

int GPS_checksum_ (GPS_UBX_t *ubx) {
  uint8_t CK_A = 0;
  uint8_t CK_B = 0;

  // Get start of checksum frame
  uint8_t *frame = (uint8_t *) ubx->cmd;

  // 8-bit Fletcher Algo
  for (size_t i = 0; i < GPS_PREAMBLE_LEN_ + ubx->cmd->len; i++) {
    CK_A += frame[i];
    CK_B += CK_A;
  }

  // Compute checksum
  if (ubx->CK_A == 0 && ubx->CK_B == 0) {
    ubx->CK_A = CK_A;
    ubx->CK_B = CK_B;

    return 1;
  }
    // Check validity of existing checksum
  else if (ubx->CK_A != CK_A) return -1;
  else if (ubx->CK_B != CK_B) return -2;
  else return 1;
}