/*
 * tty.c
 *
 *  Created on: Jul 31, 2020
 *      Author: CM Geldenhuys
 */
#include "tty.h"
#include <stdlib.h>
#include <string.h>

int __greet();
void __clearCommandBuffer();
int __processCommand();
int __PS();

int TTY_init(UART_HandleTypeDef* uart)
{
	tty.serial = (Serial_t*) malloc(sizeof(Serial_t));
	Serial_wrap(tty.serial, uart);
#ifdef TTY_GREETING
	__greet();
#endif
	__clearCommandBuffer();
	if(!*tty.PS) tty.PS = SERIAL_EOL "> ";
	__PS();
	return 1;
}

void TTY_deint()
{
	free(tty.serial);
}

int __greet()
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

			__processCommand();
			__clearCommandBuffer();
			__PS();
		}
	}
	return 1;
}

int TTY_registerCommand(const char* command, int (*func)(int argc, char *argv[]))
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

int __processCommand()
{
	//TODO: Split command string into command + args
	for(size_t idx = 0; idx < TTY_CMD_LST; idx++){
		// Lookup command
		TTY_Command_t *cmd = &tty.commandList[idx];

		// Match command string
		if(strcmp((char*) tty.command.buffer, cmd->command) == 0){
			return (*(cmd->func))(0, NULL);
		}
	}
	// Failed to find command
	return 0;
}

void __clearCommandBuffer()
{
	//TODO: Move buffer to heap (malloc) so can process multiple commands without risk of overwrite
	tty.command.len = 0;
	tty.command.pos = tty.command.buffer;
}

int __PS()
{
	return Serial_print(tty.serial, tty.PS);
}
