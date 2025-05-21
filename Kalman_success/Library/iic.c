#include "iic.h"
#include "delay.h"

// ======= 物理接线映射 =======
// 根据你实际连线改这三行：
// 本例假设：SCL→PB3，SDA→PB4
#define IIC_SCL_PORT  GPIOB
#define IIC_SCL_PIN   GPIO_Pin_4
#define IIC_SDA_PORT  GPIOB
#define IIC_SDA_PIN   GPIO_Pin_3
// =============================

#define IIC_SCL(x)    GPIO_WriteBit(IIC_SCL_PORT, IIC_SCL_PIN, (BitAction)(x))
#define IIC_SDA(x)    GPIO_WriteBit(IIC_SDA_PORT, IIC_SDA_PIN, (BitAction)(x))
#define READ_SDA()    GPIO_ReadInputDataBit(IIC_SDA_PORT, IIC_SDA_PIN)

void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    // 只用到 PB3 和 PB4，打开 PB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  // 开漏输出
    GPIO_Init(IIC_SCL_PORT, &GPIO_InitStructure);

    // 总线空闲拉高
    IIC_SCL(1);
    IIC_SDA(1);
}

void IIC_Start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    delay_us(4);
    IIC_SDA(0);
    delay_us(4);
    IIC_SCL(0);
}

void IIC_Stop(void)
{
    IIC_SCL(0);
    IIC_SDA(0);
    delay_us(4);
    IIC_SCL(1);
    IIC_SDA(1);
    delay_us(4);
}

void IIC_Send_Byte(uint8_t txd)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        IIC_SDA((txd & 0x80) >> 7);
        txd <<= 1;
        delay_us(2);
        IIC_SCL(1);
        delay_us(2);
        IIC_SCL(0);
        delay_us(2);
    }
}

uint8_t IIC_Wait_Ack(void)
{
    uint16_t errTime = 0;
    // 释放 SDA，等待从设备拉低
    IIC_SDA(1);
    delay_us(1);
    IIC_SCL(1);
    delay_us(1);
    while (READ_SDA())
    {
        if (++errTime > 250)  // 超时
        {
            IIC_Stop();
            return 1;  // 没有收到 ACK
        }
    }
    // 产生一个时钟，拉低 ACK
    IIC_SCL(0);
    return 0;  // 收到 ACK
}

void IIC_Ack(void)
{
    IIC_SCL(0);
    IIC_SDA(0);
    delay_us(2);
    IIC_SCL(1);
    delay_us(2);
    IIC_SCL(0);
}

void IIC_NAck(void)
{
    IIC_SCL(0);
    IIC_SDA(1);
    delay_us(2);
    IIC_SCL(1);
    delay_us(2);
    IIC_SCL(0);
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t receive = 0;
    IIC_SDA(1);  // SDA 置为输入
    for (uint8_t i = 0; i < 8; i++)
    {
        IIC_SCL(0);
        delay_us(2);
        IIC_SCL(1);
        receive <<= 1;
        if (READ_SDA()) receive |= 0x01;
        delay_us(1);
    }
    if (ack)
        IIC_Ack();
    else
        IIC_NAck();
    return receive;
}
