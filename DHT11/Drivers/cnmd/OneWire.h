#ifndef _ONEWIRE_H_
#define _ONEWIRE_H_

#include "stm32f1xx_hal.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} onewire_t;

void onewire_init(onewire_t* ow);
void onewire_reset(onewire_t* ow);
void onewire_write_byte(onewire_t* ow, uint8_t byte);
uint8_t onewire_read_byte(onewire_t* ow);

#endif // _ONEWIRE_H_
