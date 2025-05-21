// mpu6050.c
#include "mpu6050.h"
#include "sys.h"
#include "delay.h"
#include "iic.h"

/* 如果你用 PA15 拉 AD0，请确保在 sys.h 已定义 PAout(n) 宏 */
#define MPU_AD0_CTRL      PAout(15)

/* MPU6050 在 I2C 总线上的 7-bit 地址 (AD0=0) */
#define MPU_ADDR          0x68

/**********************************************
 * 函数名: MPU_Init
 * 描  述: 初始化 MPU6050
 * 返回值: 0 成功；1 失败
 **********************************************/
u8 MPU_Init(void)
{
    u8 res;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 1) 使能 AFIO 和 GPIOA 时钟，关闭 JTAG 复用 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* 2) 配置 PA15 为推挽输出，以控制 AD0 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    MPU_AD0_CTRL = 0;   // AD0 = 0 → I2C 地址 0x68

    /* 3) 初始化软件 I2C */
    IIC_Init();

    /* 4) 复位并唤醒 MPU6050 */
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x80);  // reset device
    delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x00);  // wake up

    /* 5) 配置量程和采样率 */
    MPU_Set_Gyro_Fsr(3);      // 2000 dps
    MPU_Set_Accel_Fsr(0);     // 2 g
    MPU_Set_Rate(50);         // 50 Hz

    /* 6) 关闭中断、FIFO 等 */
    MPU_Write_Byte(MPU_INT_EN_REG,    0x00);
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0x00);
    MPU_Write_Byte(MPU_FIFO_EN_REG,   0x00);
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0x80);

    /* 7) 读取 WHO_AM_I 验证 */
    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if (res != MPU_ADDR) {
        return 1;
    }

    /* 8) 选择时钟源为 X 轴陀螺，重新设置采样率 */
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0x01);
    MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0x00);
    MPU_Set_Rate(50);

    return 0;
}

/**********************************************
 * 配置陀螺仪量程：fsr=0→±250dps,1→±500,2→±1000,3→±2000
 **********************************************/
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3);
}

/**********************************************
 * 配置加速度量程：fsr=0→±2g,1→±4g,2→±8g,3→±16g
 **********************************************/
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3);
}

/**********************************************
 * 配置低通滤波：lpf 单位 Hz
 **********************************************/
u8 MPU_Set_LPF(u16 lpf)
{
    u8 data = 0;
    if (lpf >= 188) data = 1;
    else if (lpf >= 98)  data = 2;
    else if (lpf >= 42)  data = 3;
    else if (lpf >= 20)  data = 4;
    else if (lpf >= 10)  data = 5;
    else data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data);
}

/**********************************************
 * 配置采样率：rate 4~1000 Hz
 **********************************************/
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;
    if (rate > 1000) rate = 1000;
    if (rate < 4)    rate = 4;
    data = 1000 / rate - 1;
    MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);
    return MPU_Set_LPF(rate / 2);
}

/**********************************************
 * 读取温度，返回值为实际温度×100
 **********************************************/
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;

    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw  = (u16)buf[0] << 8 | buf[1];
    temp = 36.53f + (float)raw / 340.0f;
    return (short)(temp * 100);
}

/**********************************************
 * 读取原始陀螺数据
 **********************************************/
u8 MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0) {
        *gx = (short)((u16)buf[0] << 8 | buf[1]);
        *gy = (short)((u16)buf[2] << 8 | buf[3]);
        *gz = (short)((u16)buf[4] << 8 | buf[5]);
    }
    return res;
}

/**********************************************
 * 读取原始加速度数据
 **********************************************/
u8 MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    u8 buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0) {
        *ax = (short)((u16)buf[0] << 8 | buf[1]);
        *ay = (short)((u16)buf[2] << 8 | buf[3]);
        *az = (short)((u16)buf[4] << 8 | buf[5]);
    }
    return res;
}

/**********************************************
 * I2C 写多个字节
 **********************************************/
u8 MPU_Write_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    for (i = 0; i < len; i++) {
        IIC_Send_Byte(buf[i]);
        if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    }
    IIC_Stop();
    return 0;
}

/**********************************************
 * I2C 读多个字节
 **********************************************/
u8 MPU_Read_Len(u8 addr, u8 reg, u8 len, u8 *buf)
{
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 0);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((addr << 1) | 1);
    IIC_Wait_Ack();
    while (len) {
        if (len == 1) *buf = IIC_Read_Byte(0);
        else          *buf = IIC_Read_Byte(1);
        buf++; len--;
    }
    IIC_Stop();
    return 0;
}

/**********************************************
 * I2C 写一个字节
 **********************************************/
u8 MPU_Write_Byte(u8 reg, u8 data)
{
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 1; }
    IIC_Stop();
    return 0;
}

/**********************************************
 * I2C 读一个字节
 **********************************************/
u8 MPU_Read_Byte(u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 0);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((MPU_ADDR << 1) | 1);
    IIC_Wait_Ack();
    res = IIC_Read_Byte(0);
    IIC_Stop();
    return res;
}
