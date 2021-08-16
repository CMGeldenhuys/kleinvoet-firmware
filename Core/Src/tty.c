/*
 * tty.c
 *
 *  Created on: Jul 31, 2020
 *      Author: CM Geldenhuys
 */
#include "tty.h"
#include "logger.h"
#include <stdlib.h>
#include <string.h>
#include "oli.h"

#ifdef TTY_PRINTF

#include <stdio.h>
#include <stdarg.h>
#include "adc.h"
#include "main.h"

char workBuf_[TTY_SCREEN_WIDTH];
#endif

int TTY_greet_ ();

void TTY_clearCommandBuffer_ ();

int TTY_processCommand_ (char *cmdStrBuffer);

int TTY_PS_ ();

void TTY_splitArgs_ (char *cmdBuf, char **cmd, int *argc, char *args[]);

#ifdef DEBUG

int TTY_testArgs (int argc, char *args[]);

#endif

TTY_t tty = {0};

int TTY_init (UART_HandleTypeDef *uart)
{
  tty.serial = (Serial_t *) malloc(sizeof(Serial_t));
  Serial_wrap(tty.serial, uart);
  TTY_clearCommandBuffer_();

  TTY_greet_();

#ifdef LOG_DEST_TTY
  LOG_init();
#endif

  TTY_registerCommand("greet", &TTY_greet_);

  // Register TTY default commands
  TTY_registerCommand("reset", &CMD_resetDevice);

#ifdef DEBUG
  TTY_registerCommand("args", &TTY_testArgs);
  TTY_registerCommand("tag", &CMD_comment);
#endif

  if (!*tty.PS) {

#ifdef DEBUG
#ifdef WAVE_MOCK_WRITES
    tty.PS = SERIAL_EOL "[DEBUG+MOCK]> ";
#else // !WAVE_MOCK_WRITES
    tty.PS = SERIAL_EOL "[DEBUG]> ";
#endif // WAVE_MOCK_WRITES
#else // !DEBUG
#ifdef WAVE_MOCK_WRITES
    tty.PS = SERIAL_EOL "[MOCK]> ";
#else // !WAVE_MOCK_WRITES
    tty.PS = SERIAL_EOL "> ";
#endif // WAVE_MOCK_WRITES
#endif // DEBUG
  }
  TTY_PS_();
  return 1;
}

void TTY_deint ()
{
  free(tty.serial);
}

__weak int TTY_greet_ ()
{
  TTY_print(TTY_VT100_CLEAR_SCREEN TTY_VT100_COLOR_DEFAULT);
  Serial_write(tty.serial, oli, sizeof(oli));
  TTY_println("VERSION: " KV_VERSION);
  TTY_println("AUTHORS: " AUTHORS);

  TTY_printf("UUID: 0x%08X (%08X-%08X-%08X)" TTY_EOL, KLEINVOET_UUID, STM32_UUID[0], STM32_UUID[1], STM32_UUID[2]);
  return 1;
}

int TTY_yield ()
{
  if (Serial_available(tty.serial) > 0) {
    char recv = (char) Serial_read(tty.serial);
    // Check if printable character
    // ' ' <= recv <= '~'
    if (32u <= recv && recv <= 126u) {
      // Store rx char
      tty.command.len++;
      *(tty.command.pos++) = recv;

      // Echo back
#ifndef TTY_ECHO_DISABLE
      Serial_write(tty.serial, (uint8_t *) &recv, 1);
#endif
    }
      // Detect Backspace
    else if (recv == 127u && tty.command.len > 0) {
      tty.command.len--;
      tty.command.pos--;

#ifndef TTY_ECHO_DISABLE
      // Send Backspace & clear to the right
      Serial_print(tty.serial, SERIAL_BS TTY_VT100_CLEAR_LINE_RIGHT);
#endif
    }
      // Check for return key
      // End of command
    else if (recv == '\r') {
#ifdef DEBUG
      Serial_println(tty.serial, "");
      Serial_print(tty.serial, "< ");
      Serial_write(tty.serial, tty.command.buffer, tty.command.len);
      Serial_println(tty.serial, "");
#endif
      // Terminate command string
      (*tty.command.pos) = '\0';

      TTY_println("");
      TTY_processCommand_((char *) tty.command.buffer);
      TTY_clearCommandBuffer_();
      TTY_PS_();
    }
  }
  return 1;
}

//TODO: Make inline
int TTY_println (const char *str)
{
  return Serial_println(tty.serial, str);
}

int TTY_print (const char *str)
{
  return Serial_print(tty.serial, str);
}

int TTY_write (uint8_t *buf, size_t len)
{
  return Serial_write(tty.serial, buf, len);
}

size_t TTY_available ()
{
  return Serial_available(tty.serial);
}

uint8_t TTY_read ()
{
  return Serial_read(tty.serial);
}

int TTY_registerCommand (const char *alias, REPL_cmd_t func)
{
  static size_t idx = 0;
  // Run out of command space
  if (idx >= TTY_CMD_LST) return 0;

  tty.commandList[idx++] = (TTY_Command_t) {
          .name = alias,
          .func = func
  };
  return (int) idx;
}

#ifdef TTY_PRINTF

int TTY_printf (const char *fmt, ...)
{
  va_list args;
  va_start(args, fmt);

  int len = vsnprintf(workBuf_, TTY_SCREEN_WIDTH, fmt, args);
  // Clamp length
  TTY_write((uint8_t *) workBuf_,
            len > TTY_SCREEN_WIDTH ? TTY_SCREEN_WIDTH : len);

  va_end(args);
  return len > TTY_SCREEN_WIDTH ? -1 * len : len;
}

#endif

int TTY_processCommand_ (char *cmdStrBuffer)
{
  //TODO: Split command string into command + args
  char *cmdStr;
  char *args[TTY_ARGS_LEN];
  int  argc;

  TTY_splitArgs_(cmdStrBuffer, &cmdStr, &argc, args);

  // Command null or empty
  if (!cmdStr || strcmp(cmdStr, "") == 0) return 0;

  for (size_t idx = 0; idx < TTY_CMD_LST; idx++) {
    // Lookup command
    TTY_Command_t *cmd = &tty.commandList[idx];

    // Match command string
    if (strcmp(cmdStr, cmd->name) == 0) {
      INFO("Command received '%s'", cmd->name);
      return (*(cmd->func))(argc, args);
    }
  }
  TTY_printf("No such command '%s' found", cmdStr);
  return 0;
}

void TTY_splitArgs_ (char *cmdBuf, char **cmd, int *argc, char *args[])
{
  *cmd  = strtok(cmdBuf, TTY_ARGS_SPLIT_TOKEN);
  *argc = 0;
  char *ptr;
  while ((ptr = strtok(NULL, TTY_ARGS_SPLIT_TOKEN))
         && *argc < TTY_ARGS_LEN) {
    DBUG("arg [%d] -> %s", *argc, ptr);
    args[(*argc)++] = ptr;
  }
}

void TTY_clearCommandBuffer_ ()
{
  //TODO: Move buffer to heap (malloc) so can process multiple commands without risk of overwrite
  tty.command.len = 0;
  tty.command.pos = tty.command.buffer;
}

int TTY_PS_ ()
{
  return Serial_print(tty.serial, tty.PS);
}

int CMD_resetDevice (__unused int argc, __unused char **argv)
{
  TTY_println("System is being reset!");
  WARN("System is being reset!");
  TTY_processCommand_("sync");
  HAL_NVIC_SystemReset();
  return 1;
}

int TTY_testArgs (int argc, char *args[])
{
  TTY_println("ARGS: ");
  for (int i = 0; i < argc; i++) {
    TTY_printf("  [%i] -> '%s'" TTY_EOL, i, args[i]);
  }

  return 1;
}


#ifdef LOG_DEST_TTY

int LOG_write (uint8_t *buf, size_t len)
{
  return Serial_write(tty.serial, buf, len);
}

int LOG_flush ()
{
  // TODO: Fix once buffered writer is implemented
  return 1;
}

#endif
