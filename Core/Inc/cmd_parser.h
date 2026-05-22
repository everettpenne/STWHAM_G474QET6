/*
 * cmd_parser.h
 *
 *  Created on: May 22, 2026
 *      Author: everettpenne
 *
 *
 * Tokenises a received line and dispatches to the matching command handler.
 * Adding a new command requires only a new entry in the command_table in
 * cmd_parser.c — no header change needed.
 */

#ifndef INC_CMD_PARSER_H_
#define INC_CMD_PARSER_H_

#include "uart.h"

/**
 * dispatch_command()
 *
 * Splits buf on the first space to extract command name and argument string,
 * then calls the matching handler.  buf is modified in-place (strtok).
 * If no handler matches, sends "ERR 1 Unknown command\r\n".
 */
void dispatch_command(uart_instance_t *inst, char *buf);

#endif /* INC_CMD_PARSER_H_ */
