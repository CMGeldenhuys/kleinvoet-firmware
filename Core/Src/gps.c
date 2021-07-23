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
#include "adc.h"
#include "timestamp.h"

static GPS_t             gps = {0};
extern RTC_HandleTypeDef hrtc;
extern ADC_t             adc;

void GPS_packMsg_ (GPS_UBX_msg_t *ubx);

int GPS_checksum_ (GPS_UBX_msg_t *ubx);

size_t GPS_cmdLen_ (const GPS_UBX_cmd_t *cmd);

int GPS_configureUBX_ ();

int GPS_processCmd_ (const GPS_UBX_cmd_t *cmd);

int GPS_processCmdNav_ (const GPS_UBX_cmd_t *cmd);

GPS_rx_state_e GPS_parseByte_ (uint8_t c, GPS_rx_state_e state, GPS_rx_cmd_buffer_t * buff);

inline GPS_rx_state_e GPS_rxByte (Serial_t * serial, GPS_rx_state_e state, GPS_rx_cmd_buffer_t * buff);

void GPS_logRxState (GPS_rx_state_e state);

int GPS_updateRTC_ (RTC_HandleTypeDef *rtc, const UBX_NAV_TIMEUTC_t *cmd);

int leap (int year);

int zeller (int year, int month, int day);

int dow (int year, int month, int day);

int GPS_init (UART_HandleTypeDef *uart)
{
  DBUG("Wrapping serial");
  if (Serial_wrap(&gps.serial, uart) <= 0) return -1;

  if (GPS_configureUBX_() <= 0) return -3;
  return 1;
}

int GPS_configureUBX_ ()
{
  DBUG("Size of DEFAULT_CONFIG: %u", GPS_LEN_DEFAULT_CONFIG);
  gps.rxState  = GPS_RX_RESET;

  for (size_t i = 0; i < GPS_LEN_DEFAULT_CONFIG; i++) {
    GPS_sendCommand(GPS_DEFAULT_CONFIG[i], 100, 3);
    // NB: NEED ACK QUEUE...
    // TODO Check ACK
    // Wait between messages to ensure success
    HAL_Delay(5);
  }
  gps.state    = GPS_IDLE;
  return 1;
}

int GPS_sendCommand (const GPS_UBX_cmd_t *txCmd, int waitAck, int retryOnNack)
{
  do {
      GPS_UBX_msg_t msg = {0};
      msg.cmd = txCmd;
      DBUG("Sending UBX command 0x%02X - 0x%02X (%u bytes)", txCmd->cls, txCmd->id, txCmd->len);
      INFO("< UBX 0x%02X|0x%02X (%uB)", txCmd->cls, txCmd->id, txCmd->len);
      GPS_packMsg_(&msg);

      // Send sync header
      Serial_write(&gps.serial, (uint8_t *) &msg, 2);
      // Send command
      Serial_write(&gps.serial, (uint8_t *) txCmd, GPS_cmdLen_(txCmd));
      // Send checksum
      Serial_write(&gps.serial, (uint8_t *) &msg.CK_A, 2);

      if (waitAck > 0) {
        const uint32_t t0 = HAL_GetTick();
        // Clear state machine
        // Disassociate commands received from before command sent
        gps.rxState = GPS_RX_RESET;
        // Loop till ack or timeout
        for (;;) {
          if (Serial_available(&gps.serial) > 0) {
            GPS_rx_state_e nextState = GPS_rxByte(&gps.serial, gps.rxState, &gps.rxBuff);

            // Move to next state
            gps.rxState = GPS_RX_ADVANCE_STATE(nextState);

            const GPS_UBX_cmd_t *rxCmd = &gps.rxBuff.cmd._t;
            if (nextState == GPS_RX_CHECKSUM_PASS) {
              DBUG("Processing Command");
              GPS_processCmd_(rxCmd);
              // Check if Command is an ACK
              if (rxCmd->cls == UBX_ACK && rxCmd->id == UBX_ACK_ACK) {
                // Cast to ACK
                const UBX_ACK_t *rxCmd_t = (const UBX_ACK_t *) rxCmd;
                // Check if it is an ACK for the message sent
                if (rxCmd_t->msgClsID == txCmd->cls && rxCmd_t->msgID == txCmd->id) {
                  DBUG("ACK!");
                  return 1;
                }
                else {
                  DBUG("ACK Wrong message?");
                }
              }
              else if (rxCmd->cls == UBX_ACK && rxCmd->id == UBX_ACK_NACK) {
                DBUG("NACKed");
                break;
              }
            }

            // Check if timeout has expired
            const uint32_t delT = HAL_GetTick() - t0;
            if (delT > waitAck) {
              WARN("ACK Timeout (%d)", delT);
              break;
            }
          }
          // If no data in serial wait a bit
          else HAL_Delay(1);
        }
      }
  } while (retryOnNack --> 0);
  return -1;
}

inline size_t GPS_cmdLen_ (const GPS_UBX_cmd_t *cmd)
{
  return GPS_PREAMBLE_LEN_ + cmd->len;
}

int GPS_yield ()
{
  // Process data received over serial

  if (gps.state & GPS_CONFIG) {
    gps.state &= ~GPS_CONFIG;
    if (GPS_configureUBX_() > 0) {
      DBUG("GPS configured");
      gps.state |= GPS_IDLE;
      return 1;
    }
  }

  if (Serial_available(&gps.serial) > 0) {
    // Receive and parse byte
    GPS_rx_state_e nextState = GPS_rxByte(&gps.serial, gps.rxState, &gps.rxBuff);

    // Move to next state
    gps.rxState = GPS_RX_ADVANCE_STATE(nextState);

    // If checksum PASSED process CMD
    if (nextState == GPS_RX_CHECKSUM_PASS) {
      DBUG("Processing Command");
      return GPS_processCmd_(&gps.rxBuff.cmd._t);
    }
  }

  return 1;
}

void GPS_logRxState (GPS_rx_state_e state)
{
  switch (state) {
    case GPS_RX_UBX_DET:
      INFO("UBX Message Detected");
      break;

    case GPS_RX_NEMA_DET:
      INFO("NMEA Message Detected");
      break;

    case GPS_RX_POTENTIAL_COMMAND_OK:
      INFO("SYNC bytes received");
      break;

    case GPS_RX_POTENTIAL_COMMAND_UNEXPECTED:
      INFO("Unexpected byte");
      break;

    case GPS_RX_PREAMBLE_COMPLETE:
      INFO("Preamble complete");
      break;

    case GPS_RX_PREAMBLE_PENDING:
      INFO("Preamble Pending");
      break;

    case GPS_RX_PREAMBLE_OVERFLOW:
      INFO("Preamble Overflow");
      break;

    case GPS_RX_PAYLOAD_OK:
      INFO("Payload OK");
      break;

    case GPS_RX_PAYLOAD_PENDING:
      INFO("Payload Pending");
      break;

    case GPS_RX_PAYLOAD_OVERFLOW:
      INFO("Payload Overflow");
      break;

    case GPS_RX_CK_A_ACK:
      INFO("Checksum(A) received");
      break;

    case GPS_RX_CHECKSUM_PASS:
      INFO("Checksum PASS");
      break;

    case GPS_RX_CHECKSUM_FAIL:
      INFO("Checksum FAIL");
      break;

    case GPS_RX_NO_DATA:
    case GPS_RX_RESET:
      break;

    default:
      INFO("Unknown State (0x%08X)", state);
      break;
  }
}

inline GPS_rx_state_e GPS_rxByte (Serial_t * serial, GPS_rx_state_e state, GPS_rx_cmd_buffer_t * buff)
{
  // Check if Byte available
  if (Serial_available(serial) > 0) {
    // Get byte from serial
    const uint8_t  c         = Serial_peek(serial);
    DBUG("--------(0x%02X)--------", c);

    GPS_rx_state_e nextState = GPS_parseByte_(c, state, buff);
    // Only advance serial `head` if byte was successfully parsed
    if (nextState > 0) {
      Serial_read(serial);
    }
#ifdef GPS_DEBUG_SERIAL
    GPS_logRxState(nextState);
#endif
    return nextState;
  }
  else return GPS_RX_NO_DATA;
}

GPS_rx_state_e GPS_parseByte_ (uint8_t c, GPS_rx_state_e state, GPS_rx_cmd_buffer_t * buff)
{
  switch (state) {

    // Sync 1
    default:
    case GPS_RX_RESET:
    case GPS_RX_WAIT: {
      // Received start byte
      if (c == GPS_SYNC_1_) return GPS_RX_UBX_DET;
      // Potential start of NMEA message
      else if (c == GPS_NMEA_ID) {
        INFO("Potential NMEA message detected");
#ifdef GPS_PARSE_NMEA
//        HAL_UART_DMAStop(gps.serial.config_.uart);
        // Store pointer to start of message
        char *nmeaMessage = (char *) Serial_tail(&gps.serial);
        const size_t nBytes = Serial_available(&gps.serial);
        // While data left of serial
        for (size_t i = 0; i < nBytes; i++) {
          // Get next char but don't move on pointer
          uint8_t nextVal = *(nmeaMessage + i);
          if (nextVal == GPS_SYNC_1_) {
            INFO("Aborting NMEA message detection (UBX Sync detected)");
            Serial_advanceTailRx(&gps.serial, i);
            return 1;
          }
            // CR detected
            // TODO: check for LF too but for now CR is enough
          else if (nextVal == '\r') {
            *(nmeaMessage + i) = '\0'; // NULL terminate string
            INFO("> NMEA: %s", nmeaMessage);
            return 1;
          }
        }
        // Neither outcome resolved (not UBX nor complete NMEA)
        WARN("NMEA message not complete during processing");
        return -1;
#else
        return GPS_RX_NEMA_DET;
#endif
      }
      else return GPS_RX_WAITING;
      break;
    }

      // Sync 2
    case GPS_RX_POTENTIAL_COMMAND: {
      // Received second byte
      if (c == GPS_SYNC_2_) {
        DBUG("New UBX command recv");

        // Reset internal state
        buff->idx = 0;
        memset(buff->cmd.mem, 0, GPS_BUF_LEN);

        return GPS_RX_POTENTIAL_COMMAND_OK;
      }
      else return GPS_RX_POTENTIAL_COMMAND_UNEXPECTED;
      break;
    }
      // Preamble
    case GPS_RX_PREAMBLE: {
#ifdef GPS_DEBUG_SERIAL
      DBUG("Recv preamble \t[%03u] <- 0x%02X", gps.rx.idx, c);
#endif
      // Store Rx char to buff
      buff->cmd.mem[buff->idx++] = c;

      // Finished getting preamble
      if (buff->idx >= GPS_PREAMBLE_LEN_) {
#ifdef GPS_DEBUG_SERIAL
        DBUG("CMD Len: %u", gps.rx.cmd._t.len);
#endif
        buff->idx = 0;
        return GPS_RX_PREAMBLE_COMPLETE;
      }
      // Check overflow
      else if (buff->idx >= GPS_BUF_LEN) {
        WARN("Command buffer overflow!");
        return GPS_RX_PREAMBLE_OVERFLOW;
      }
      // Preamble incomplete
      else return GPS_RX_PREAMBLE_PENDING;

      break;
    }

    case GPS_RX_PAYLOAD: {
#ifdef GPS_DEBUG_SERIAL
      DBUG("Recv payload \t[%03u] <- 0x%02X", gps.rx.idx, c);
#endif
      // Store Payload
      buff->cmd._t.payload[buff->idx++] = c;

      if (buff->idx == buff->cmd._t.len) return GPS_RX_PAYLOAD_OK;
      else if (buff->idx >= GPS_MAX_CMD_LEN) {
        WARN("Command buffer overflow!");
        return GPS_RX_PAYLOAD_OVERFLOW;
      }
      else return GPS_RX_PAYLOAD_PENDING;
      break;
    }
      // Ck_a
    case GPS_RX_CK_A: {
#ifdef GPS_DEBUG_SERIAL
      DBUG("Recv CK_A (0x%02X)", c);
#endif
      buff->CK_A  = c;
      return GPS_RX_CK_A_ACK;
      break;
    }
      // Ck_b
    case GPS_RX_CK_B: {
#ifdef GPS_DEBUG_SERIAL
      DBUG("Recv CK_B (0x%02X)", c);
#endif
      buff->CK_B  = c;
      // Internal state transition
      // NB!: Run into next state
    }
      // FUll cmd
    case GPS_RX_CHECKSUM: {
      DBUG("---- Full Command Recv ----");

      // Pack message
      GPS_UBX_msg_t tmp = {0};
      tmp.cmd           = &buff->cmd._t;
      tmp.CK_A          = buff->CK_A;
      tmp.CK_B          = buff->CK_B;
      if (GPS_checksum_(&tmp) > 0) {
        DBUG("Checksum status: PASSED");
        return GPS_RX_CHECKSUM_PASS;
      }
      else {
        DBUG("Checksum status: FAILED");
        return GPS_RX_CHECKSUM_FAIL;
      }
      break;
    }
  }

  return GPS_RX_UNKNOWN;
}

int GPS_processCmd_ (const GPS_UBX_cmd_t *cmd)
{
  DBUG("> UBX 0x%02X|0x%02X (%uB)", cmd->cls, cmd->id, cmd->len);
  switch (cmd->cls) {
    case UBX_NAV: {
      DBUG("> NAV (0x%02X | 0x%02X)", cmd->cls, cmd->id);
      return GPS_processCmdNav_(cmd);
    }

    case UBX_ACK: {
      const UBX_ACK_t *cmd_t = (const UBX_ACK_t *) cmd;

      if(cmd->id == UBX_ACK_ACK){
        INFO("> ACK  (0x%02X | 0x%02X)", cmd_t->msgClsID, cmd_t->msgID);
        // TODO: Process ACK msg to make sure commands are successful
      }
      // NACK
      else {
        WARN("> NACK (0x%02X | 0x%02X)", cmd_t->msgClsID, cmd_t->msgID);
        // TODO: Process ACK msg to make sure commands are successful
      }
      return 1;
    }

    default: {
      WARN("GPS Msg ignored (0x%02X | 0x%02X)", cmd->cls, cmd->id);
      return 0;
    }
  }
}

int GPS_processCmdNav_ (const GPS_UBX_cmd_t *cmd)
{
  if (cmd->cls != UBX_NAV) {
    WARN("Not Nav cmd");
    return 0;
  }

  switch (cmd->id) {

    case UBX_NAV_CLK: {
      // Parse Command
      const UBX_NAV_CLK_t *cmd_t = (UBX_NAV_CLK_t *) &gps.rxBuff.cmd._t;

      // Log Message
      GPS_log_UBX_NAV_CLK(cmd_t);

      return UBX_NAV_CLK;
    }

    case UBX_NAV_TIMEUTC: {
      // Parse serial command into struct
      const UBX_NAV_TIMEUTC_t *cmd_t = (UBX_NAV_TIMEUTC_t *) &gps.rxBuff.cmd._t;
      // Log out command received
      GPS_log_UBX_NAV_TIMEUTC(cmd_t);

      // If current cmd's time is valid
      if (cmd_t->valid & UBX_NAV_TIMEUTC_VALIDUTC) {
        // Keep track of number of valid samples within one status window
        gps.timeValidN++;

        // Time stamp previously marked sample with time received from GPS
        TIME_stamp(cmd_t);

        // Previously not valid
        if (!gps.timeValid) {
          // Update RTC with current GPS time
          GPS_updateRTC_(&hrtc, cmd_t);

          // Once fix & valid time then disable SAT msgs
          GPS_sendCommand(&GPS_DISABLE_UBX_NAV_SAT.generic, 0, 0);

          // Mark time as valid for next epoch
          gps.timeValid = 1;
          // Notify timestamping that time is locked
          TIME_timeValid();
        }
      }
      else
      {
        // Keep track of the number of invalid time message in one status window
        gps.timeInvalidN++;

        // If current time is NOT valid and previous time is valid
        // If fix lost ensure SAT is on as to show feedback of num sats
        if (gps.timeValid) {
          GPS_sendCommand(&GPS_ENABLE_UBX_NAV_SAT.generic, 0, 0);
          gps.timeValid = 0;
          TIME_timeInvalid();
        }
        // If time was never valid and is still not valid
        else {
          DBUG("Not valid and was previously not valid");
        }
      }



      return UBX_NAV_TIMEUTC;
    }

    case UBX_NAV_STATUS: {
      const UBX_NAV_STATUS_t *cmd_t = (UBX_NAV_STATUS_t *) &gps.rxBuff.cmd._t;

      // Log message
      GPS_log_UBX_NAV_STATUS(cmd_t);

      // Reset counters for next status window
      gps.timeInvalidN = 0;
      gps.timeValidN = 0;

      return UBX_NAV_STATUS;
    }

    case UBX_NAV_SAT: {
      // Parse command
      const UBX_NAV_SAT_t *cmd_t = (UBX_NAV_SAT_t *) &gps.rxBuff.cmd._t;

      // Log message
      GPS_log_UBX_NAV_SAT(cmd_t);

      return UBX_NAV_SAT;
    }

    case UBX_NAV_HPPOSECEF: {
      // Parse Command
      const UBX_NAV_HPPOSECEF_t *cmd_t = (UBX_NAV_HPPOSECEF_t *) &gps.rxBuff.cmd._t;

      // Log message
      GPS_log_UBX_NAV_HPPOSECEF(cmd_t);

      return UBX_NAV_HPPOSECEF;
    }

    case UBX_NAV_POSECEF: {
      // Parse Command
      const UBX_NAV_POSECEF_t *cmd_t = (UBX_NAV_POSECEF_t *) &gps.rxBuff.cmd._t;

      // Log message
      GPS_log_UBX_NAV_POSECEF(cmd_t);

      // Check if time is valid
      if(gps.timeValid) {
        // Store location to WAVE header
        ADC_updateLocation(&(cmd_t->ecefX), cmd_t->pAcc);
      }
      else {
        WARN("Time invalid NOT updating location");
      }

      return UBX_NAV_POSECEF;
    }

    case UBX_NAV_PVT: {
      // Parse Command
      const UBX_NAV_PVT_t *cmd_t = (UBX_NAV_PVT_t *) cmd;

      // Log message
      GPS_log_UBX_NAV_PVT(cmd_t);

      return UBX_NAV_HPPOSECEF;
    }

    default: {
      INFO("NAV Msg ignored (0x%02X)", cmd->id);
      return 0;
    }

  }
}


void GPS_packMsg_ (GPS_UBX_msg_t *ubx)
{
  // Place synchronisation characters for bus speed
  ubx->sync_1 = GPS_SYNC_1_;
  ubx->sync_2 = GPS_SYNC_2_;

  GPS_checksum_(ubx);
}

int GPS_checksum_ (GPS_UBX_msg_t *ubx)
{
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

int GPS_updateRTC_ (RTC_HandleTypeDef *rtc, const UBX_NAV_TIMEUTC_t *cmd)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  sTime.Hours          = cmd->hour;
  sTime.Minutes        = cmd->min;
  sTime.Seconds        = cmd->sec;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(rtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
    return -1;
  }

  sDate.Year    = cmd->year % 2000; // Will probably break but not now... or at least soon xD
  sDate.Month   = RTC_ByteToBcd2(cmd->month);
  sDate.Date    = cmd->day;
  sDate.WeekDay = dow(cmd->year, cmd->month, cmd->day);

  if (HAL_RTC_SetDate(rtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
    return -2;
  }

  INFO("Successfully update RTC with GPS time");

  return 1;


}

// Taken from https://electronics.stackexchange.com/questions/66285/how-to-calculate-day-of-the-week-for-rtc
// Author: Dave Tweed

/* Returns the number of days to the start of the specified year, taking leap
 * years into account, but not the shift from the Julian calendar to the
 * Gregorian calendar. Instead, it is as though the Gregorian calendar is
 * extrapolated back in time to a hypothetical "year zero".
 */
int leap (int year)
{
  return year * 365 + (year / 4) - (year / 100) + (year / 400);
}

/* Returns a number representing the number of days since March 1 in the
 * hypothetical year 0, not counting the change from the Julian calendar
 * to the Gregorian calendar that occured in the 16th century. This
 * algorithm is loosely based on a function known as "Zeller's Congruence".
 * This number MOD 7 gives the day of week, where 0 = Monday and 6 = Sunday.
 */
int zeller (int year, int month, int day)
{
  year += ((month + 9) / 12) - 1;
  month = (month + 9) % 12;
  return leap(year) + month * 30 + ((6 * month + 5) / 10) + day + 1;
}

/* Returns the day of week (1=Monday, 7=Sunday) for a given date.
 */
int dow (int year, int month, int day)
{
  return (zeller(year, month, day) % 7) + 1;
}