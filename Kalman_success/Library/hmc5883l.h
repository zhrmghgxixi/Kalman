#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "stm32f10x.h"

// 初始化 HMC5883L 磁力计
void HMC5883L_Init(void);
// 读取磁力计原始数据 (单位 LSB)
void HMC5883L_Read(int16_t *mx, int16_t *my, int16_t *mz);

#endif  // __HMC5883L_H