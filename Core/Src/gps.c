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

int GPS_processCmd_ (GPS_UBX_cmd_t *cmd);

int GPS_processCmdNav_ (const GPS_UBX_cmd_t *cmd);

int GPS_rxByte_ (uint8_t c);

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
  for (size_t i = 0; i < GPS_LEN_DEFAULT_CONFIG; i++) {
    GPS_sendCommand(GPS_DEFAULT_CONFIG[i], 0, 0);
    // NB: NEED ACK QUEUE...
    // TODO Check ACK
    // Wait between messages to ensure success
    HAL_Delay(10);
  }
  gps.rx.state = GPS_IDLE;
  gps.state    = GPS_IDLE;
  return 1;
}

int GPS_sendCommand (const GPS_UBX_cmd_t *cmd, __unused int waitAck, __unused int retryOnNack)
{
  // TODO: Implement ACk & NACK to retry
  GPS_UBX_msg_t msg = {0};
  msg.cmd = cmd;
  DBUG("Sending UBX command 0x%02X - 0x%02X (%u bytes)", cmd->cls, cmd->id, cmd->len);
  INFO("< UBX 0x%02X|0x%02X (%uB)", cmd->cls, cmd->id, cmd->len);
  GPS_packMsg_(&msg);

  // Send sync header
  Serial_write(&gps.serial, (uint8_t *) &msg, 2);
  // Send command
  Serial_write(&gps.serial, (uint8_t *) cmd, GPS_cmdLen_(cmd));
  // Send checksum
  Serial_write(&gps.serial, (uint8_t *) &msg.CK_A, 2);

  return 1;
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
  // Check if byte ready for RX
  gps.state |= Serial_available(&gps.serial) > 0 ? GPS_RX : 0u;
  if (gps.state & GPS_RX) {
    // Remove RX flag
    gps.state &= ~GPS_RX;
    // Place device in idle
    gps.state |= GPS_IDLE;
    return GPS_rxByte_(Serial_read(&gps.serial));
  }

  return 1;
}

int GPS_rxByte_ (uint8_t c)
{
  switch (gps.rx.state) {

    // Sync 1
    case GPS_IDLE: {
      // Cache sample when time code was received
      // TODO: ADC fix up
//      gps.adcTimestamp = adc.sampleCount;

      // Received start byte
      if (c == GPS_SYNC_1_) gps.rx.state = GPS_RX_SYNC_2;
      // Potential start of NMEA messga
      else if (c == '$') {
        INFO("Potential NMEA message detected");
        // Store pointer to start of message
        char *nmeaMessage = (char *) SERIAL_HEAD(&gps.serial);
        // While data left of serial
        while (Serial_available(&gps.serial) > 0) {
          // Get next char but don't move on pointer
          uint8_t nextVal = Serial_peek(&gps.serial);
          if (nextVal == GPS_SYNC_1_) {
            INFO("Aborting NMEA message detection (UBX Sync detected)");
            break;
          }
            // CR detected
            // TODO: check for LF too but for now CR is enough
          else if (nextVal == '\r') {
            *SERIAL_HEAD(&gps.serial) = '\0'; // NULL terminate string

            // Move on HEAD
            Serial_read(&gps.serial);

            INFO("NMEA message detected! -> %s", nmeaMessage);
            break;
          }
          // Move on HEAD
          Serial_read(&gps.serial);
        }
      }
      break;
    }

      // Sync 2
    case GPS_RX_SYNC_2: {
      // Received second byte
      if (c == GPS_SYNC_2_) {
        DBUG("New UBX command recv");

        // Reset internal state
        gps.rx.idx = 0;
        memset(gps.rx.cmd.mem, 0, GPS_BUF_LEN);

        gps.rx.state = GPS_RX_PREAMBLE;
      }
      else gps.rx.state = GPS_IDLE;
      break;
    }
      // Preamble
    case GPS_RX_PREAMBLE: {
#ifdef GPS_DEBUG_SERIAL
      DBUG("Recv preamble \t[%03u] <- 0x%02X", gps.rx.idx, c);
#endif
      gps.rx.cmd.mem[gps.rx.idx++] = c;

      // Finished getting preamble
      if (gps.rx.idx >= GPS_PREAMBLE_LEN_) {
#ifdef GPS_DEBUG_SERIAL
        DBUG("CMD Len: %u", gps.rx.cmd._t.len);
#endif
        gps.rx.state = GPS_RX_PAYLOAD;
        gps.rx.idx   = 0;
      }

      if (gps.rx.idx >= GPS_BUF_LEN) {
        WARN("Command buffer overflow!");
        gps.rx.state = GPS_IDLE;
      }
      break;
    }

    case GPS_RX_PAYLOAD: {
#ifdef GPS_DEBUG_SERIAL
      DBUG("Recv payload \t[%03u] <- 0x%02X", gps.rx.idx, c);
#endif
      gps.rx.cmd._t.payload[gps.rx.idx++] = c;
      if (gps.rx.idx >= gps.rx.cmd._t.len) gps.rx.state = GPS_RX_CK_A;
      if (gps.rx.idx >= GPS_BUF_LEN - GPS_PREAMBLE_LEN_) {
        WARN("Command buffer overflow!");
        gps.rx.state = GPS_IDLE;
      }
      break;
    }
      // Ck_a
    case GPS_RX_CK_A: {
#ifdef GPS_DEBUG_SERIAL
      DBUG("Recv CK_A (0x%02X)", c);
#endif
      gps.rx.CK_A  = c;
      gps.rx.state = GPS_RX_CK_B;
      break;
    }
      // Ck_b
    case GPS_RX_CK_B: {
#ifdef GPS_DEBUG_SERIAL
      DBUG("Recv CK_B (0x%02X)", c);
#endif
      gps.rx.CK_B = c;

      gps.rx.state = GPS_RX_CHECKSUM;
      // Run into next state
    }
      // FUll cmd
    case GPS_RX_CHECKSUM: {
      DBUG("---- Full Command Recv ----");
      GPS_UBX_msg_t tmp = {0};
      tmp.cmd           = &gps.rx.cmd._t;
      tmp.CK_A          = gps.rx.CK_A;
      tmp.CK_B          = gps.rx.CK_B;
      if (GPS_checksum_(&tmp) > 0) {
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
      gps.rx.state = GPS_IDLE;
      return GPS_processCmd_(&gps.rx.cmd._t);
      // Yield end of command
      // Allow time for other commands
    }

    default: {
      gps.rx.state = GPS_IDLE;
      WARN("Unkown state (placing into idle)");
      break;
    }
  }

  return 1;
}

int GPS_processCmd_ (GPS_UBX_cmd_t *cmd)
{
  DBUG("> UBX 0x%02X|0x%02X (%uB)", cmd->cls, cmd->id, cmd->len);
  switch (cmd->cls) {
    case UBX_NAV: {
      DBUG("> NAV (0x%02X | 0x%02X)", cmd->cls, cmd->id);
      return GPS_processCmdNav_(cmd);
    }

    case UBX_ACK: {
      const UBX_ACK_t *cmd_t = (UBX_ACK_t *) cmd;

      if(cmd->id == UBX_ACK_ACK){
        INFO("> ACK (0x%02X | 0x%02X)", cmd_t->msgClsID, cmd_t->msgID);
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
      const UBX_NAV_CLK_t *cmd_t = (UBX_NAV_CLK_t *) &gps.rx.cmd._t;

      // Log Message
      GPS_log_UBX_NAV_CLK(cmd_t);

      return UBX_NAV_CLK;
    }

    case UBX_NAV_TIMEUTC: {
      // Parse serial command into struct
      const UBX_NAV_TIMEUTC_t *cmd_t = (UBX_NAV_TIMEUTC_t *) &gps.rx.cmd._t;
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
      const UBX_NAV_STATUS_t *cmd_t = (UBX_NAV_STATUS_t *) &gps.rx.cmd._t;

      // Log message
      GPS_log_UBX_NAV_STATUS(cmd_t);

      // Reset counters for next status window
      gps.timeInvalidN = 0;
      gps.timeValidN = 0;

      return UBX_NAV_STATUS;
    }

    case UBX_NAV_SAT: {
      // Parse command
      const UBX_NAV_SAT_t *cmd_t = (UBX_NAV_SAT_t *) &gps.rx.cmd._t;

      // Log message
      GPS_log_UBX_NAV_SAT(cmd_t);

      return UBX_NAV_SAT;
    }

    case UBX_NAV_HPPOSECEF: {
      // Parse Command
      const UBX_NAV_HPPOSECEF_t *cmd_t = (UBX_NAV_HPPOSECEF_t *) &gps.rx.cmd._t;

      // Log message
      GPS_log_UBX_NAV_HPPOSECEF(cmd_t);

      return UBX_NAV_HPPOSECEF;
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