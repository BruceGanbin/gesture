#ifndef _LOG_2_H
#define _LOG_2_H

#include "stm32f10x.h"
#define BUFFER_SIZE             256

#define CMD_LOG_TYPE_HIGH       0
#define CMD_LOG_TYPE_LOW        1
#define SYS_LOG_TYPE            2
#define CMD_HIGH_HALF_SIZE      20
#define CMD_LOW_HALF_SIZE       18


void log_printf(char* format,...);
void log_disable(void);
void log_enable(void);

void get_cmd(void);
#endif

