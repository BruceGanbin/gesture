#include "log2.h"
#include "stm32f10x_usart.h"
#include "stm32f10x.h"
#include "stm32_serial.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

typedef enum {
    LOG_DISABLE = 0,
    LOG_ENABLE
}log_start_t;

static char buffer[BUFFER_SIZE];
static log_start_t log_start = LOG_ENABLE;

static void sendstring(char *str)
{
    while(*str!='\0')
    {
        while(!(USART1->SR & USART_FLAG_TXE));
        USART1->DR = (*str & 0x1FF);
        str++;
    }
}


void log_printf(char* format,...)
{
    va_list ap;
    if(log_start == LOG_ENABLE)
    {
        va_start(ap,format);
        vsprintf(buffer,format,ap);
        sendstring(buffer);
        va_end(ap);
    }

}

void log_disable(void)
{
    log_start = LOG_DISABLE;
}

void log_enable(void)
{
    log_start = LOG_ENABLE;
}

void get_cmd(void)
{
    unsigned char data[64]={0};
    static unsigned char cmd[64] = {0};
    static unsigned char cmd_count=0;
    unsigned char len = 0;
    unsigned char i = 0;
    len = serial_read(COM1,data,64);
    if(len == 0) return;
    log_printf("%s",data);
    while(i<len)
    {
        if((data[i]==0x0A) || (data[i]==0x0D))
            break;
        cmd[cmd_count] = data[i];
        i++;
        cmd_count++;
        if(i==len) return;
    }
    cmd_count = 0;
    log_printf("\r\ncommand :%s\r\n",cmd);
    memset(cmd,0,sizeof(cmd));
}

unsigned char get_debug_data(unsigned char* buf,unsigned char len)
{
    return serial_read(COM1,buf,len);
}
