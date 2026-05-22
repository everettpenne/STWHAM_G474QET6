/*
 * commands.c
 *
 *  Created on: May 22, 2026
 *      Author: everettpenne
 *
 * Command handler implementations.
 *
 * Response conventions
 * ─────────────────────
 *   OK\r\n              — accepted, no data
 *   OK <value>\r\n      — accepted, with return value
 *   ERR <n> <msg>\r\n   — rejected; error codes are stable across versions
 *
 * Error codes
 * ────────────
 *   1   Unknown command
 *   2   Pulse active or in lockout
 *   3   Ramp arrays not loaded
 *   4   Not in a runnable mode
 *   5   Missing or invalid argument
 *   6   Mode transition rejected
 *   7   Frequency out of range
 *   8   Duty cycle out of range
 *   9   Pulse length out of range
 *  10   Array receive timeout or hardware error
 *  11   Array size mismatch
 */

#include "commands.h"
#include "uart.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

void cmd_idn(uart_instance_t *inst, char *args)
{
    (void)args;
    uart_send(inst, "OK HVPS G474QET6 v0.2\r\n");
}

void cmd_rst(uart_instance_t *inst, char *args)
{
	/* Add reset logic */
    uart_send(inst, "OK No reset.\r\n");
}

void cmd_helloworld(uart_instance_t *inst, char *args)
{
    (void)args;
    uart_send(inst, "Hello, world!\r\n");
}

