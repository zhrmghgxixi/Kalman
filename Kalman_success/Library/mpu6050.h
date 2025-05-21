#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"
#include "iic.h"       // 你先前写的 iic.h
#include "delay.h"     // delay_us() / delay_ms()

// MPU6050 在 I2C 总线上的 7-bit 地址：
// 如果 AD0 拉低用 0x68；如果 AD0 拉高用 0x69
#define MPU_ADDR            0x68  

// 把原来驱动里用到的 MPU_IIC_xxx 映射到你的 IIC_xxx
#define MPU_IIC_Init()      IIC_Init()
#define MPU_IIC_Start()     IIC_Start()
#define MPU_IIC_Stop()      IIC_Stop()
#define MPU_IIC_Send_Byte   IIC_Send_Byte
#define MPU_IIC_Read_Byte   IIC_Read_Byte
#define MPU_IIC_Wait_Ack    IIC_Wait_Ack
#define MPU_IIC_Ack         IIC_Ack
#define MPU_IIC_NAck        IIC_NAck

// 原代码里用到的寄存器地址宏，保留不动
#define MPU_DEVICE_ID_REG   0x75
#define MPU_PWR_MGMT1_REG   0x6B
#define MPU_PWR_MGMT2_REG   0x6C
#define MPU_GYRO_CFG_REG    0x1B
#define MPU_ACCEL_CFG_REG   0x1C
#define MPU_CFG_REG         0x1A
#define MPU_SAMPLE_RATE_REG 0x19
#define MPU_INT_EN_REG      0x38
#define MPU_USER_CTRL_REG   0x6A
#define MPU_FIFO_EN_REG     0x23
#define MPU_INTBP_CFG_REG   0x37

#define MPU_TEMP_OUTH_REG   0x41
#define MPU_ACCEL_XOUTH_REG 0x3B
#define MPU_GYRO_XOUTH_REG  0x43

// …以下就保留你那份 .c 里所有函数声明…
u8 MPU_Init(void);
u8 MPU_Set_Gyro_Fsr(u8 fsr);
u8 MPU_Set_Accel_Fsr(u8 fsr);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU_Set_Rate(u16 rate);
short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU_Write_Byte(u8 reg,u8 data);
u8 MPU_Read_Byte(u8 reg);

#endif
