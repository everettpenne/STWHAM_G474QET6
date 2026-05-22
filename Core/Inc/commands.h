/*
 * commands.h
 *
 *  Created on: May 22, 2026
 *      Author: everettpenne
 *
 * Command handler declarations.
 *
 * Command set
 * ────────────
 * Every handler follows:  void cmd_xxx(uart_instance_t *inst, char *args)
 *
 * Response conventions
 * ─────────────────────
 *   OK\r\n              — accepted, no return value
 *   OK <value>\r\n      — accepted, scalar or string return value
 *   ERR <n> <msg>\r\n   — rejected; <n> is a stable numeric code
 *
 * Full command reference
 * ──────────────────────
 *
 *  *IDN               → "OK HVPS G474QET6 v0.2"
 *  *RST               → stop all outputs, reset state, "OK"
 *
 *  HELLO              → "Hello, world!" (connection test)
 *
 */

#ifndef INC_COMMANDS_H_
#define INC_COMMANDS_H_

#include "uart.h"

/* Identification / system */
void cmd_idn(uart_instance_t *inst, char *args);
void cmd_rst(uart_instance_t *inst, char *args);

/* Misc */
void cmd_helloworld(uart_instance_t *inst, char *args);

#endif /* INC_COMMANDS_H_ */
