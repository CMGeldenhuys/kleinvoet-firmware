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

void GPS_packMsg_ (GPS_UBX_msg_t *ubx);

int GPS_checksum_ (GPS_UBX_msg_t *ubx);

size_t GPS_cmdLen_ (const GPS_UBX_cmd_t *cmd);

int GPS_sendCommand (const GPS_UBX_cmd_t *cmd);

int GPS_configureUBX_ ();

int GPS_init (UART_HandleTypeDef *uart) {
  Serial_wrap(&gps.serial, uart);

  // TODO: Move to clear buffer method
  gps.rx.idx = 0;

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
  GPS_UBX_msg_t msg = {0};
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

int GPS_yield () {
    // Process data received over serial
  while (Serial_available(&gps.serial) > 0) {
    uint8_t c = Serial_read(&gps.serial);

    DBUG("recv: 0x%02X", c);

    switch(gps.rx.state) {

        // Sync 1
      case GPS_IDLE: {
        // Received start byte
        if (c == GPS_SYNC_1_) gps.rx.state = GPS_RX_SYNC_2;
        break;
      }

      // Sync 2
      case GPS_RX_SYNC_2: {
          DBUG("New command recv");
          // Received second byte
          if (c == GPS_SYNC_2_) gps.rx.state = GPS_RX_PREAMBLE;
          break;
      }
        // Preable
      case GPS_RX_PREAMBLE: {
          DBUG("Recv preamble [%u]", gps.rx.idx);
        gps.rx.cmd.mem[gps.rx.idx++] = c;

        // Finished getting preamble
        if(gps.rx.idx >= GPS_PREAMBLE_LEN_) {
            DBUG("CMD Len: %u", gps.rx.cmd._t.len);
            gps.rx.state = GPS_RX_PAYLOAD;
            gps.rx.idx = 0;
        }
        break;
      }

      case GPS_RX_PAYLOAD: {
          DBUG("Recv payload [%u]", gps.rx.idx);
          gps.rx.cmd._t.payload[gps.rx.idx++] = c;
          if(gps.rx.idx >= gps.rx.cmd._t.len ) gps.rx.state = GPS_RX_CK_A;
          break;
      }
        // Ck_a
      case GPS_RX_CK_A: {
        DBUG("Recv CK_A");
        gps.rx.CK_A = c;
        gps.rx.state = GPS_RX_CK_B;
        break;
      }
        // Ck_b
      case GPS_RX_CK_B: {
        DBUG("Recv CK_B");
        gps.rx.CK_B = c;
        gps.rx.state = GPS_RX_DONE;
        break;
      }
        // FUll cmd
      case GPS_RX_DONE: {
          DBUG("---- Full Command Recv ----");
          GPS_UBX_msg_t tmp = {0};
          tmp.cmd = &gps.rx.cmd._t;
          tmp.CK_A = gps.rx.CK_A;
          tmp.CK_B = gps.rx.CK_B;
          DBUG("Checksum status: %d",GPS_checksum_(&tmp));
          gps.rx.state = GPS_IDLE;
          break;
      }

      default:
        break;
    }
  }
}

void GPS_packMsg_ (GPS_UBX_msg_t *ubx) {
  // Place synchronisation characters for bus speed
  ubx->sync_1 = GPS_SYNC_1_;
  ubx->sync_2 = GPS_SYNC_2_;

  GPS_checksum_(ubx);
}

int GPS_checksum_ (GPS_UBX_msg_t *ubx) {
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
  else return 2;
}