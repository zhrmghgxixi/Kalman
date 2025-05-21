#include "onewire.h"

void onewire_init(onewire_t* ow)
{
    // 初始化引脚为输出
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = ow->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ow->port, &GPIO_InitStruct);
}

void onewire_reset(onewire_t* ow)
{
    // 实现OneWire复位序列
    HAL_GPIO_WritePin(ow->port, ow->pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(ow->port, ow->pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

void onewire_write_byte(onewire_t* ow, uint8_t byte)
{
    // 实现写一个字节数据到OneWire设备
    for (int i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(ow->port, ow->pin, GPIO_PIN_RESET);
        if (byte & 0x01)
        {
            HAL_GPIO_WritePin(ow->port, ow->pin, GPIO_PIN_SET);
        }
        HAL_Delay(1);
        HAL_GPIO_WritePin(ow->port, ow->pin, GPIO_PIN_SET);
        HAL_Delay(1);
        byte >>= 1;
    }
}

uint8_t onewire_read_byte(onewire_t* ow)
{
    // 实现从OneWire设备读取一个字节数据
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(ow->port, ow->pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(ow->port, ow->pin, GPIO_PIN_SET);
        HAL_Delay(1);
        byte >>= 1;
        if (HAL_GPIO_ReadPin(ow->port, ow->pin))
        {
            byte |= 0x80;
        }
    }
    return byte;
}
