/*
 * cmd_parser.c
 *
 *  Created on: May 22, 2026
 *      Author: everettpenne
 *
 * cmd_parser.c
 *
 * Tokenises received lines and dispatches to command handlers.
 *
 * To add a command
 * ─────────────────
 *  1. Implement handler in commands.c
 *  2. Declare in commands.h
 *  3. Add a { "NAME", handler } row to command_table[] below
 *  Nothing else changes.
 */

#include "cmd_parser.h"
#include "commands.h"
#include "uart.h"
#include <string.h>

typedef void (*cmd_handler_t)(uart_instance_t *inst, char *args);

typedef struct {
    const char    *name;
    cmd_handler_t  handler;
} command_t;

static const command_t command_table[] = {
    /* Identification */
    { "*IDN",         cmd_idn          },
    { "*RST",         cmd_rst          },

    /* Misc */
    { "HELLO",        cmd_helloworld   },
};

#define NUM_COMMANDS  (sizeof(command_table) / sizeof(command_table[0]))

void dispatch_command(uart_instance_t *inst, char *buf)
{
    char *cmd  = strtok(buf, " \r\n");
    char *args = strtok(NULL, "\r\n");

    if (cmd == NULL) return;

    for (int i = 0; i < (int)NUM_COMMANDS; i++) {
        if (strcmp(cmd, command_table[i].name) == 0) {
            command_table[i].handler(inst, args);
            return;
        }
    }

    uart_send(inst, "ERR 1 Unknown command\r\n");
}


