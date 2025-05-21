#include "hmc5883l.h"
#include "iic.h"
#include "delay.h"

// HMC5883L I2C 7 位地址
#define HMC_ADDR        0x1E
#define HMC_CONFIG_A    0x00
#define HMC_CONFIG_B    0x01
#define HMC_MODE        0x02
#define HMC_DATA_X_MSB  0x03

/**
 * @brief  初始化 HMC5883L：连续测量模式，默认增益
 */
void HMC5883L_Init(void)
{
    uint8_t cfgA = 0x70;  // 8 次采样，15Hz，正常测量
    uint8_t cfgB = 0x20;  // 增益 1.3 Ga (默认)
    uint8_t mode = 0x00;  // 连续测量模式

    MPU_Write_Len(HMC_ADDR, HMC_CONFIG_A, 1, &cfgA);
    MPU_Write_Len(HMC_ADDR, HMC_CONFIG_B, 1, &cfgB);
    MPU_Write_Len(HMC_ADDR, HMC_MODE,     1, &mode);
    delay_ms(6);
}

/**
 * @brief  读取磁力计 X/Z/Y 三轴原始值
 */
void HMC5883L_Read(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t buf[6];
    MPU_Read_Len(HMC_ADDR, HMC_DATA_X_MSB, 6, buf);
    *mx = (int16_t)(buf[0] << 8 | buf[1]);
    *mz = (int16_t)(buf[2] << 8 | buf[3]);
    *my = (int16_t)(buf[4] << 8 | buf[5]);
}