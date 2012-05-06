#ifndef RN42_H
#define RN42_H

#include "serial.h"

void
rn42_init(
    volatile avr32_usart_t *usart,
    uint32_t irq,
    uint32_t gpio_pin);


void
rn42_task(void);


#endif /* RN42_H */
