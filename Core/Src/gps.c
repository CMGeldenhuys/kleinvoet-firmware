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
        else if (c == '$') DBUG("NMEA Msg recv");
        break;
      }

      // Sync 2
      case GPS_RX_SYNC_2: {
          // Received second byte
          if (c == GPS_SYNC_2_){
              DBUG("New command recv");

              // Reset internal state
              gps.rx.idx = 0;
              memset(gps.rx.cmd.mem, 0, GPS_BUF_LEN);

              gps.rx.state = GPS_RX_PREAMBLE;
          }
          else gps.rx.state = GPS_IDLE;
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

        gps.rx.state = GPS_RX_CHECKSUM;
        // Run into next state
      }
        // FUll cmd
      case GPS_RX_CHECKSUM: {
          DBUG("---- Full Command Recv ----");
          GPS_UBX_msg_t tmp = {0};
          tmp.cmd = &gps.rx.cmd._t;
          tmp.CK_A = gps.rx.CK_A;
          tmp.CK_B = gps.rx.CK_B;
          if(GPS_checksum_(&tmp) > 0) {
              DBUG("Checksum status: PASSED");
              gps.rx.state = GPS_RX_PROCESS_CMD;
              // Run into next state
          }
          else {
              DBUG("Checksum status: FAILED");
              gps.rx.state = GPS_IDLE;
              return 0;
          }

      }

      case GPS_RX_PROCESS_CMD: {
          switch(gps.rx.cmd._t.cls) {
              case UBX_NAV:{
                  INFO("NAV Msg recv");
                  switch(gps.rx.cmd._t.id) {
                      // UBX-NAV-CLK
                      case 0x22:{
                          UBX_NAV_CLK_t *cmd = (UBX_NAV_CLK_t *) &gps.rx.cmd._t;
                          DBUG("\tiTOW: %lu", cmd->iTOW);
                          DBUG("\tclkB: %l", cmd->clkB);
                          DBUG("\tclkD: %l", cmd->clkD);
                          DBUG("\ttAcc: %lu", cmd->tAcc);
                          DBUG("\tfAcc: %lu", cmd->fAcc);
                        break;
                      }

                      // UBX-NAV-STATUS
                      case 0x03:{
                          UBX_NAV_STATUS_t *cmd = (UBX_NAV_STATUS_t *) &gps.rx.cmd._t;
                          DBUG("\tiTOW: %lu", cmd->iTOW);
                          DBUG("\tgpsFix: 0x%02X", cmd->gpsFix);
                          DBUG("\tflags: 0x%02X", cmd->flags);
                          DBUG("\tfixStat: 0x%02x", cmd->fixStat);
                          DBUG("\tflags2: 0x%02X", cmd->flags2);
                          DBUG("\tttff: %lu", cmd->ttff);
                          DBUG("\tmsss: %lu", cmd->msss);
                          if(cmd->gpsFix != 0x00) {
                              GPS_sendCommand(&GPS_DISABLE_UBX_NAV_SAT.generic);
                              GPS_sendCommand(&GPS_ENABLE_UBX_NAV_CLK.generic);
                          }
                          break;
                      }

                      // UBX-NAV-SAT
                      case 0x35: {
                          UBX_NAV_SAT_t *cmd = (UBX_NAV_SAT_t *) &gps.rx.cmd._t;
                          DBUG("\tiTOW: %lu", cmd->iTOW);
                          DBUG("\tversion: %u", cmd->version);
                          DBUG("\tnumSvs: %u", cmd->numSvs);

                          for(uint8_t i = 0; i < cmd->numSvs; i++){
                              DBUG("\t---- Svs %u ----", i);
                              DBUG("\t\tgnssId: %u", cmd->svs[i].gnssId);
                              DBUG("\t\tsvId: %u", cmd->svs[i].svId);
                              DBUG("\t\tcno: %u", cmd->svs[i].cno);
                              DBUG("\t\telev: %d", cmd->svs[i].elev);
                              DBUG("\t\tazim: %d", cmd->svs[i].azim);
                              DBUG("\t\tprRes: %d", cmd->svs[i].prRes);
                              DBUG("\t\tflags: %lu", cmd->svs[i].flags);

                          }
                          break;
                      }

                      default:{
                          DBUG("Unknown Msg id");
                          break;
                      }

                  }
                  break;
              }

              default: {
                  INFO("GPS Msg ignored (0x%02X | 0x%02X)", gps.rx.cmd._t.cls, gps.rx.cmd._t.id);
                  break;
              }

          }

          gps.rx.state = GPS_IDLE;
          return 1;
          // Yield end of command
          // Allow time for other commands
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