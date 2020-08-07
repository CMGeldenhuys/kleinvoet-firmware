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

#ifdef TTY_PRINTF
#include <stdio.h>
#include <stdarg.h>
char workBuf_[TTY_SCREEN_WIDTH];
#endif

int TTY_greet_();
void TTY_clearCommandBuffer_();
int TTY_processCommand_();
int TTY_PS_();

TTY_t tty = {0};

int TTY_init(UART_HandleTypeDef* uart)
{
	tty.serial = (Serial_t*) malloc(sizeof(Serial_t));
	Serial_wrap(tty.serial, uart);
  TTY_clearCommandBuffer_();
#ifdef TTY_GREETING
  TTY_greet_();
  TTY_registerCommand("greet", &TTY_greet_);
#endif
	if(!*tty.PS) tty.PS = SERIAL_EOL "> ";
  TTY_PS_();
	return 1;
}

void TTY_deint()
{
	free(tty.serial);
}

int TTY_greet_()
{
	return Serial_println(tty.serial, TTY_GREETING);
}

int TTY_yield()
{
	if( Serial_available(tty.serial) > 0 ) {
		char recv = (char) Serial_read(tty.serial);
		// Check if printable character
		// ' ' <= recv <= '~'
		if(32u <= recv && recv <= 126u){
			// Store rx char
			tty.command.len++;
			*(tty.command.pos ++) = recv;

			// Echo back
#ifndef TTY_ECHO_DISABLE
			Serial_write(tty.serial, (uint8_t*) &recv, 1);
#endif
		}
		// Detect Backspace
		else if(recv == 127u && tty.command.len > 0){
			tty.command.len--;
			tty.command.pos--;

#ifndef TTY_ECHO_DISABLE
			// Send Backspace
			Serial_print(tty.serial, SERIAL_BS);
#endif
		}
		// Check for return key
		// End of command
		else if(recv == '\r') {
#ifdef DEBUG
			Serial_println(tty.serial, "");
			Serial_print(tty.serial, "< ");
			Serial_write(tty.serial, tty.command.buffer, tty.command.len);
			Serial_println(tty.serial, "");
#endif
			// Terminate command string
			(*tty.command.pos) = '\0';

      TTY_processCommand_();
      TTY_clearCommandBuffer_();
      TTY_PS_();
		}
	}
	return 1;
}

//TODO: Make inline
int TTY_println(const char *str)
{
  return Serial_println(tty.serial, str);
}

int TTY_print(const char *str)
{
  return Serial_print(tty.serial, str);
}

int TTY_write(uint8_t *buf, size_t len)
{
  return Serial_write(tty.serial, buf, len);
}

int TTY_available()
{
  return Serial_available(tty.serial);
}

uint8_t TTY_read()
{
  return Serial_read(tty.serial);
}

int TTY_registerCommand(const char *command, int (*func)(int argc, char *argv[]))
{
	static size_t idx = 0;
	// Run out of command space
	if(idx >= TTY_CMD_LST) return 0;

	tty.commandList[idx++] = (TTY_Command_t){
			.command = command,
			.func = func
	};
	return idx;
}

#ifdef TTY_PRINTF
int TTY_printf(const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);

  int len = vsnprintf(workBuf_, TTY_SCREEN_WIDTH, fmt, args);
  // Clamp length
  TTY_write((uint8_t*)workBuf_,
            len > TTY_SCREEN_WIDTH ?  TTY_SCREEN_WIDTH : len);
  return len > TTY_SCREEN_WIDTH ? -1 * len : len;
}
#endif

int TTY_processCommand_()
{
	//TODO: Split command string into command + args
	for(size_t idx = 0; idx < TTY_CMD_LST; idx++){
		// Lookup command
		TTY_Command_t *cmd = &tty.commandList[idx];

		// Match command string
		if(strcmp((char*) tty.command.buffer, cmd->command) == 0){
		  // TODO: send args
		  DBUG("Command received '%s'", cmd->command)
			return (*(cmd->func))(0, NULL);
		}
	}
	// Failed to find command
	// TODO: Report
	return 0;
}

void TTY_clearCommandBuffer_()
{
	//TODO: Move buffer to heap (malloc) so can process multiple commands without risk of overwrite
	tty.command.len = 0;
	tty.command.pos = tty.command.buffer;
}

int TTY_PS_()
{
	return Serial_print(tty.serial, tty.PS);
}

int LOG_altWrite( uint8_t *buf,  size_t len)
{
  return Serial_write(tty.serial, buf, len);
}
