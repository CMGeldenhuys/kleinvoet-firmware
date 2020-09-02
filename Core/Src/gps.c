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

int GPS_configureUBX_ ();

int GPS_init (UART_HandleTypeDef *uart)
{
  Serial_wrap(&gps.serial, uart);
  GPS_configureUBX_();

  HAL_Delay(1000);

  return 1;
}

int GPS_configureUBX_ ()
{
  DBUG("Size of DEFAULT_CONFIG: %u", GPS_LEN_DEFAULT_CONFIG);
  for (size_t i = 0; i < GPS_LEN_DEFAULT_CONFIG; i++) {
    GPS_sendCommand(GPS_DEFAULT_CONFIG[i]);
  }

  return 1;
}

int GPS_sendCommand (const GPS_UBX_cmd_t *cmd)
{
  // TODO: Implement ACk & NACK to retry
  GPS_UBX_t msg = {0};
  msg.cmd = cmd;
  DBUG("Sending UBX command 0x%02x - 0x%02x", cmd->cls, cmd->id);
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


int GPS_yield ()
{

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