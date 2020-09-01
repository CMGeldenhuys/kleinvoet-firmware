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

void GPS_clearCMDBuf_ ();

int GPS_appendCMD_ (char c);

void GPS_packMsg_ (GPS_UBX_t *ubx);

void GPS_checksum_ (GPS_UBX_t *ubx);

size_t GPS_cmdLen_ (const GPS_UBX_cmd_t *cmd);

int GPS_sendCommand (const GPS_UBX_cmd_t *cmd);

GPS_UBX_cmd_t test = {
        .class = 0x06,
        .ID = 0x02,
        .len = 8u,
        .payload={0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF}
};

int GPS_init (UART_HandleTypeDef *uart)
{
  DBUG("Size of GPS_UBX_class: %u", sizeof(GPS_UBX_Class_e));
  DBUG("Size of GPS_UBX_cmd: %u", sizeof(GPS_UBX_cmd_t));
  DBUG("Size of UBX_CFG_PRT: %u", sizeof(UBX_CFG_PRT));
  Serial_wrap(&gps.serial, uart);
  GPS_sendCommand(&test);

//  GPS_sendCommand(&GPS_GET_PORT_CONFIG.generic);
  HAL_Delay(1000);

  return 1;
}

int GPS_sendCommand (const GPS_UBX_cmd_t *cmd)
{
  GPS_UBX_t msg = {0};
  msg.cmd = cmd;

  GPS_packMsg_(&msg);

  // Send sync header
  Serial_write(&gps.serial, (uint8_t *) &msg, 2);
  // Send command
  Serial_write(&gps.serial, (uint8_t *) cmd, GPS_cmdLen_(cmd));
  // Send checksum
  Serial_write(&gps.serial, (uint8_t *) &msg.CK_A, 2);
}

inline size_t GPS_cmdLen_ (const GPS_UBX_cmd_t *cmd)
{
  return GPS_PREAMBLE_LEN_ + cmd->len;
}

void GPS_clearCMDBuf_ ()
{
  gps.cmd.started = 1;
  gps.cmd.pos     = gps.cmd.buf;
}

int GPS_appendCMD_ (char c)
{
  if (gps.cmd.started > 0) {

    if (gps.cmd.buf - gps.cmd.pos < GPS_CMD_LEN) {
      *gps.cmd.pos++ = c;
    }
    else {
      WARN("CMD buffer overflow!");
      return -1;
    }
  }
  else {
    DBUG("CMD not started yet");
  }

  return 1;
}


int GPS_yield ()
{
  int bytesReady;
  while ((bytesReady = Serial_available(&gps.serial)) > 0) {
    // TODO: Implement a serial read to char (eg. read to /n)
    char currByte = (char) Serial_read(&gps.serial);
    switch (currByte) {

      case '$': {
        GPS_clearCMDBuf_();
        break;
      }

      default: {
        GPS_appendCMD_(currByte);
        break;
      }

      case '\r':
        break;

      case '\n': {
        GPS_appendCMD_('\0');
        gps.cmd.started = 0;
        INFO(gps.cmd.buf);
        break;
      }
    }
  }
  if (bytesReady < 0) DBUG("Command buffer overflow!");
}

void GPS_packMsg_ (GPS_UBX_t *ubx)
{
  // Place synchronisation characters for bus speed
  ubx->sync_1 = GPS_SYNC_1_;
  ubx->sync_2 = GPS_SYNC_2_;

  GPS_checksum_(ubx);
}

void GPS_checksum_ (GPS_UBX_t *ubx)
{
  ubx->CK_A = 0;
  ubx->CK_B = 0;

  // Get start of checksum frame
  uint8_t *frame = (uint8_t *) ubx->cmd;

  // 8-bit Fletcher Algo
  for (size_t i = 0; i < GPS_PREAMBLE_LEN_ + ubx->cmd->len; i++) {
    ubx->CK_A += frame[i];
    ubx->CK_B += ubx->CK_A;
  }
}