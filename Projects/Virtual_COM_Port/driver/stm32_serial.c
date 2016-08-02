#include "stm32f10x.h"
#include "stm32_serial.h"
//#include "hw_timer.h"
#include <string.h>

#ifdef USING_USART1
//Serial_Buf uart1_tx_buf;
Serial_Buf uart1_rx_buf;
static char uart1_rx_flag = 0;
#endif
#ifdef USING_USART2
Serial_Buf uart2_tx_buf;
Serial_Buf uart2_rx_buf;
static char uart2_rx_flag = 0;
#endif
#ifdef USING_USART3
Serial_Buf uart3_tx_buf;
Serial_Buf uart3_rx_buf;
static char uart3_rx_flag = 0;
#endif

//static unsigned int over_time = 0;

void serial_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

#ifdef USING_USART1  // usart1 to debug printf
//    memset(&uart1_tx_buf,0,sizeof(Serial_Buf));
    memset(&uart1_rx_buf,0,sizeof(Serial_Buf));

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);

    GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = USART1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = USART1_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1,&USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1,ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
#endif

#ifdef USING_USART2
    memset(&uart2_tx_buf,0,sizeof(Serial_Buf));
    memset(&uart2_rx_buf,0,sizeof(Serial_Buf));

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

    GPIO_InitStructure.GPIO_Pin = USART2_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = USART2_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = USART2_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2,&USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2,ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
#endif

#ifdef USING_USART3
    memset(&uart3_tx_buf,0,sizeof(Serial_Buf));
    memset(&uart3_rx_buf,0,sizeof(Serial_Buf));

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

    GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = USART3_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3,&USART_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3,ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
#endif

}

void serial_release(unsigned char device)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    switch(device)
    {
#ifdef USING_USART1
    case COM1:
    GPIO_InitStructure.GPIO_Pin = USART1_TX_PIN | USART1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(USART1_PORT,&GPIO_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,DISABLE);
    USART_Cmd(USART1,DISABLE);
    break;
#endif

#ifdef USING_USART2
    case COM2:
    GPIO_InitStructure.GPIO_Pin = USART2_TX_PIN | USART2_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(USART2_PORT,&GPIO_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,DISABLE);
    USART_Cmd(USART2,DISABLE);
    break;
#endif

#ifdef USING_USART3
    case COM3:
    GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN | USART3_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(USART3_PORT,&GPIO_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,DISABLE);
    USART_Cmd(USART3,DISABLE);
    break;
#endif
    default:
        break;
    }
}


unsigned short serial_read_pol(unsigned char port,unsigned char *Data,unsigned short len)
{
    USART_TypeDef *pUSART = 0;

    switch(port)
    {
#ifdef USING_USART1
    case COM1: pUSART = USART1;
        break;
#endif
#ifdef USING_USART2
    case COM2: pUSART = USART2;
        break;
#endif
#ifdef USING_USART3
    case COM3: pUSART = USART3;
        break;
#endif
    default:
        break;
    }
    USART_ITConfig(pUSART, USART_IT_RXNE, DISABLE);
    while(len--)
    {
        //        USART_ClearFlag(Port,USART_FLAG_RXNE);
        while (!(pUSART->SR & USART_FLAG_RXNE));
        *Data = pUSART->DR & 0xFF;
        Data++;
    }
    USART_ITConfig(pUSART, USART_IT_RXNE, ENABLE);
    return len;
}

unsigned short serial_send_pol(unsigned char port,unsigned char *Data,unsigned short len)
{
    USART_TypeDef *pUSART = 0;
    switch(port)
    {
#ifdef USING_USART1
    case COM1: pUSART = USART1;
        break;
#endif
#ifdef USING_USART2
    case COM2: pUSART = USART2;
        break;
#endif
#ifdef USING_USART3
    case COM3: pUSART = USART3;
        break;
#endif
    default:
        break;
    }
    while(len--)
    {
        while (!(pUSART->SR & USART_FLAG_TXE));
        pUSART->DR = *Data;
        Data++;
    }

    return len;
}


unsigned short Get_dat_len(unsigned short wp,unsigned short rp,unsigned short buf_len)
{
    unsigned short len;

    if(wp < rp)
        len = buf_len - rp + wp;
    else
        len = wp - rp;
    return len;
}

unsigned short serial_read(unsigned char port,unsigned char *Data,unsigned short buflen)
{
    //    unsigned int delt_time = 0;
    unsigned short datlen;
    unsigned short wp,rp;
    Serial_Buf* Ser_buf = 0;
    char *flag = 0;

    switch(port)
    {
#ifdef USING_USART1
    case COM1:
        Ser_buf = &uart1_rx_buf;
        flag = &uart1_rx_flag;
        break;
#endif
#ifdef USING_USART2
    case COM2:
        Ser_buf = &uart2_rx_buf;
        flag = &uart2_rx_flag;
        break;
#endif
#ifdef USING_USART3
    case COM3:
        Ser_buf = &uart3_rx_buf;
        flag = &uart3_rx_flag;
        break;
#endif
    default:
        break;
    }

    wp = Ser_buf->WriteP;
    rp = Ser_buf->ReadP;

    datlen = Get_dat_len(wp,rp,UART_BUF_LEN);
    if(datlen == 0)
        return 0;

    //   serial_read over time 50 ms
    //    delt_time = get_timer();
    //    if(delt_time - over_time < UART_TIMEOUT) return 0;
    if(*flag == 0) return 0;
    *flag = 0;

    datlen = datlen > buflen?buflen:datlen;
    if(wp > rp)
    {
        memcpy(Data, &Ser_buf->Buf[rp], datlen);
    }
    else
    {
        if(rp + datlen < UART_BUF_LEN)
        {
            memcpy(Data, &Ser_buf->Buf[rp], datlen);
        }
        else
        {
            memcpy(Data, &Ser_buf->Buf[rp], UART_BUF_LEN - rp);
            memcpy(Data, &Ser_buf->Buf[0], datlen + rp - UART_BUF_LEN);
        }
    }

    //move read point
    Ser_buf->ReadP += datlen;
    Ser_buf->ReadP %= UART_BUF_LEN;

    return datlen;
}

unsigned short serial_send(unsigned char port,unsigned char *Data,unsigned short len)
{

    Serial_Buf* Ser_buf = 0;

    switch(port)
    {
#ifdef USING_USART1
//  case COM1:
//        Ser_buf = &uart1_tx_buf;
//        break;
#endif
#ifdef USING_USART2
    case COM2:
        Ser_buf = &uart2_tx_buf;
        USART_ITConfig(USART2,USART_IT_TXE,DISABLE);
        break;
#endif
#ifdef USING_USART3
    case COM3:
        Ser_buf = &uart3_tx_buf;
        USART_ITConfig(USART3,USART_IT_TXE,DISABLE);
        break;
#endif
    default:
        break;
    }

    // cp data to ring buffer
    while(len--)
    {
        Ser_buf->Buf[Ser_buf->WriteP] = *Data;
        Data++;
        Ser_buf->WriteP++;
        Ser_buf->WriteP %= UART_BUF_LEN;
    }

    switch(port)
    {
#ifdef USING_USART1
    case COM1:
        break;
#endif
#ifdef USING_USART2
    case COM2:
        USART_ITConfig(USART2,USART_IT_TXE,ENABLE);
        break;
#endif
#ifdef USING_USART3
    case COM3:
        USART_ITConfig(USART3,USART_IT_TXE,ENABLE);
        break;
#endif
    default:
        break;
    }

    return len;
}


void Uart_ISR_rx(USART_TypeDef* Port,Serial_Buf *buf)
{
    buf->Buf[buf->WriteP] = Port->DR;
    buf->WriteP++;
    buf->WriteP %= UART_BUF_LEN;
    // get last tmie
    //    over_time = get_timer();
    //    log_printf("^%d",buf->WriteP);
}

void Uart_ISR_tx(USART_TypeDef* Port,Serial_Buf *buf)
{
    unsigned short datalen;
    unsigned short wp,rp;

    wp = buf->WriteP;
    rp = buf->ReadP;

    datalen = Get_dat_len(wp,rp,UART_BUF_LEN);
    if(datalen == 0)
    {
        USART_ITConfig(Port,USART_IT_TXE,DISABLE);
        buf->ReadP -= 1;
    }
    Port->DR = buf->Buf[rp];

    buf->ReadP++;
    buf->ReadP %= UART_BUF_LEN;
}

#ifdef USING_USART1
void USART1_IRQHandler(void)
{
    unsigned char temp = 0;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        Uart_ISR_rx(USART1,&uart1_rx_buf);
    }
    if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET)
    {
        temp = USART1->DR;
        uart1_rx_flag = 1;
    }
}
#endif

#ifdef USING_USART2
void USART2_IRQHandler(void)
{
    unsigned char temp = 0;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        Uart_ISR_rx(USART2,&uart2_rx_buf);
    }
    if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
    {
        Uart_ISR_tx(USART2,&uart2_tx_buf);
    }
    if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET)
    {
        temp = USART2->DR;
        uart2_rx_flag = 1;
    }
}
#endif

#ifdef USING_USART3
void USART3_IRQHandler(void)
{
    unsigned char temp = 0;
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        Uart_ISR_rx(USART3,&uart3_rx_buf);
    }
    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
        Uart_ISR_tx(USART3,&uart3_tx_buf);
    }
    if(USART_GetITStatus(USART3,USART_IT_IDLE) == SET)
    {
        temp = USART3->DR;
        uart3_rx_flag = 1;
    }
}
#endif
