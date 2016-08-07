#ifndef I2C_H
#define I2C_H

#include "stm32f10x.h"

#define I2C1_SDA_PIN                  GPIO_Pin_7
#define I2C1_SCL_PIN                  GPIO_Pin_6
#define I2C1_GPIO_PORT                GPIOB
#define I2C1_GPIO_CLK                 RCC_APB2Periph_GPIOB
#define I2C1_CLK                      RCC_APB1Periph_I2C1

#define I2C2_SDA_PIN                  GPIO_Pin_11
#define I2C2_SCL_PIN                  GPIO_Pin_10
#define I2C2_GPIO_PORT                GPIOB
#define I2C2_GPIO_CLK                 RCC_APB2Periph_GPIOB
#define I2C2_CLK                      RCC_APB1Periph_I2C2

#define I2C_MEM_1Byte                 1
#define I2C_MEM_2Bytes                2

#define I2C_TimeOut                   500

#define I2C_WTimeOut                  0x08
#define I2C_RTimeOut                  0x0F
#define I2C_NOTimeout                 0x00

#define I2C1_DMA_ADDR                 ((uint32_t)0x40005410)
#define I2C1_DMA_CLK                  RCC_AHBPeriph_DMA1
#define I2C1_DMA_TX_CHANNEL           DMA1_Channel6
#define I2C1_DMA_RX_CHANNEL           DMA1_Channel7

#define I2C2_DMA_ADDR                 ((uint32_t)0x40005810)
#define I2C2_DMA_CLK                  RCC_AHBPeriph_DMA1
#define I2C2_DMA_TX_CHANNEL           DMA1_Channel4
#define I2C2_DMA_RX_CHANNEL           DMA1_Channel5

#define IIC_STOP()                    I2C_GenerateSTOP(I2C1,ENABLE)

#define Rd                            0
#define Tx                            1

typedef enum
{
    DMA_TX = 0,
    DMA_RX = 1
}IIC_RT_Typedef;

void I2C_init(void);
//unsigned char IIC_Write(uint8_t *pBuffer,uint8_t PartAddr,uint8_t WriteAddr,uint16_t NumByteToRead);
//unsigned char IIC_Read(uint8_t *pBuffer,uint8_t PartAddr,uint8_t WriteAddr,uint16_t NumByteToRead);
unsigned char IIC_Read(uint8_t PartAddr,uint8_t WriteAddr,uint16_t NumByteToRead,uint8_t *pBuffer);
unsigned char IIC_Write(uint8_t PartAddr,uint8_t WriteAddr,uint16_t NumByteToWrite,uint8_t *pBuffer);
unsigned char CheckIIC_Ack(uint8_t WriteAddr);
void IIC_Reset_Bus(void);

#endif
