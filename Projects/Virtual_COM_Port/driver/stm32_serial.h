#ifndef __STM32_SERIAL_H
#define __STM32_SERIAL_H

#include "stm32f10x.h"

#define USING_USART1
//#define USING_USART2
//#define USING_USART3

#define COM1                     1
#define COM2                     2
#define COM3                     3


#define UART_BUF_LEN             128
#define UART_TIMEOUT             50
//Usart GPIO
#define USART1_RX_PIN            GPIO_Pin_10
#define USART1_TX_PIN            GPIO_Pin_9
#define USART1_RX_SOURCE         GPIO_PinSource10
#define USART1_TX_SOURCE         GPIO_PinSource9
#define USART1_PORT              GPIOA
#define USART1_BAUDRATE          115200

#define USART2_RX_PIN            GPIO_Pin_3
#define USART2_TX_PIN            GPIO_Pin_2
#define USART2_RX_SOURCE         GPIO_PinSource3
#define USART2_TX_SOURCE         GPIO_PinSource2
#define USART2_PORT              GPIOA
#define USART2_BAUDRATE          115200


#define USART3_RX_PIN            GPIO_Pin_11
#define USART3_TX_PIN            GPIO_Pin_10
#define USART3_RX_SOURCE         GPIO_PinSource11
#define USART3_TX_SOURCE         GPIO_PinSource10
#define USART3_PORT              GPIOB
#define USART3_BAUDRATE          115200


typedef struct UART_BUF
{
    unsigned char Buf[UART_BUF_LEN];
    unsigned short ReadP;
    unsigned short WriteP;
    unsigned int time_tag;
}Serial_Buf;


void serial_init(void);
void serial_release(unsigned char device);

unsigned short serial_read_pol(unsigned char port,unsigned char *Data,unsigned short len);
unsigned short serial_send_pol(unsigned char port,unsigned char *Data,unsigned short len);

unsigned short serial_read(unsigned char Port,unsigned char *Data,unsigned short len);
unsigned short serial_send(unsigned char Port,unsigned char *Data,unsigned short len);

void uart_transmit_data(unsigned char data);
void uart_transmit_message(unsigned char *data, unsigned char length);

#endif
