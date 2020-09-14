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

static GPS_t             gps = {0};
extern RTC_HandleTypeDef hrtc;

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
  Serial_wrap(&gps.serial, uart);
  GPS_configureUBX_();
  return 1;
}

int GPS_configureUBX_ ()
{
  DBUG("Size of DEFAULT_CONFIG: %u", GPS_LEN_DEFAULT_CONFIG);
  for (size_t i = 0; i < GPS_LEN_DEFAULT_CONFIG; i++) {
    GPS_sendCommand(GPS_DEFAULT_CONFIG[i], 0, 0);
  }
  gps.rx.state = GPS_IDLE;
  gps.state = GPS_IDLE;
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
      // Received start byte
      if (c == GPS_SYNC_1_) gps.rx.state = GPS_RX_SYNC_2;
      else if (c == '$') DBUG("NMEA Msg recv (ignoring)");
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
  INFO("> UBX 0x%02X|0x%02X (%uB)", cmd->cls, cmd->id, cmd->len);
  switch (cmd->cls) {
    case UBX_NAV: {
      DBUG("NAV Msg recv (0x%02X | 0x%02X)", cmd->cls, cmd->id);
      return GPS_processCmdNav_(cmd);
    }

    case UBX_ACK: {
      DBUG("ACK Msg recv (0x%02X | 0x%02X)", cmd->cls, cmd->id);
      // TODO: Process ACK msg to make sure commands are successful
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
      INFO("UBX-NAV-CLK");
      const UBX_NAV_CLK_t *cmd_t = (UBX_NAV_CLK_t *) &gps.rx.cmd._t;
      DBUG("  iTOW: %lu", cmd_t->iTOW);
      DBUG("  clkB: %l", cmd_t->clkB);
      DBUG("  clkD: %l", cmd_t->clkD);
      DBUG("  tAcc: %lu", cmd_t->tAcc);
      DBUG("  fAcc: %lu", cmd_t->fAcc);
      return UBX_NAV_CLK;
    }

    case UBX_NAV_TIMEUTC: {
      INFO("UBX-NAV-TIMEUTC");
      const UBX_NAV_TIMEUTC_t *cmd_t = (UBX_NAV_TIMEUTC_t *) &gps.rx.cmd._t;
      DBUG("  iTOW: %lu", cmd_t->iTOW);
      DBUG("  tAcc: %lu", cmd_t->tAcc);
      DBUG("  nano: %0l", cmd_t->nano);
      DBUG("  year: %u", cmd_t->year);
      DBUG("  month: %02u", cmd_t->month);
      DBUG("  day: %02u", cmd_t->day);
      DBUG("  hour: %02u", cmd_t->hour);
      DBUG("  min: %02u", cmd_t->min);
      DBUG("  sec: %02u", cmd_t->sec);
      DBUG("  valid: 0x%02X", cmd_t->valid);


      // TODO: Replace with constant blip
      if (cmd_t->valid & UBX_NAV_TIMEUTC_VALIDUTC) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        GPS_updateRTC_(&hrtc, cmd_t);
        // Once fix & valid time then disable SAT msgs
        GPS_sendCommand(&GPS_DISABLE_UBX_NAV_SAT.generic, 0, 0);
      }
      else{
        // If fix lost or never had ensure SAT is on as to show feedback of num sats
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
        GPS_sendCommand(&GPS_ENABLE_UBX_NAV_SAT.generic, 0, 0);
      }

      return UBX_NAV_TIMEUTC;
    }

    case UBX_NAV_STATUS: {
      INFO("UBX-NAV-STATUS");
      const UBX_NAV_STATUS_t *cmd_t = (UBX_NAV_STATUS_t *) &gps.rx.cmd._t;
      DBUG("  iTOW: %lu", cmd_t->iTOW);
      DBUG("  gpsFix: 0x%02X", cmd_t->gpsFix);
      DBUG("  flags: 0x%02X", cmd_t->flags);
      DBUG("  fixStat: 0x%02x", cmd_t->fixStat);
      DBUG("  flags2: 0x%02X", cmd_t->flags2);
      DBUG("  ttff: %lu", cmd_t->ttff);
      DBUG("  msss: %lu", cmd_t->msss);
      return UBX_NAV_STATUS;
    }

    case UBX_NAV_SAT: {
      INFO("UBX-NAV-SAT");
      const UBX_NAV_SAT_t *cmd_t = (UBX_NAV_SAT_t *) &gps.rx.cmd._t;
      DBUG("  iTOW: %lu", cmd_t->iTOW);
      DBUG("  version: %u", cmd_t->version);
      DBUG("  numSvs: %u", cmd_t->numSvs);

      for (uint8_t i = 0; i < cmd_t->numSvs; i++) {
        DBUG("  ---- Svs %u ----", i);
        DBUG("    gnssId: %u", cmd_t->svs[i].gnssId);
        DBUG("    svId: %u", cmd_t->svs[i].svId);
        DBUG("    cno: %u", cmd_t->svs[i].cno);
        DBUG("    elev: %d", cmd_t->svs[i].elev);
        DBUG("    azim: %d", cmd_t->svs[i].azim);
        DBUG("    prRes: %d", cmd_t->svs[i].prRes);
        DBUG("    flags: 0x%02X", cmd_t->svs[i].flags);
        // TODO: Make this non blocking??
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
      }
      return UBX_NAV_SAT;
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

  sDate.Year    = cmd->year % 2000; // Will probably break but not now...
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