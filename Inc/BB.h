#ifndef _BB_H_
#define _BB_H_

#include <stm32f0xx_hal.h>

uint32_t getId();
void sendMessage(uint8_t port, uint8_t data[], uint16_t size, uint8_t has_ack);

#endif // _BB_H_
