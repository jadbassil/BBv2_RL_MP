#ifndef _SUBLAYER_12_
#define _SUBLAYER_12_

#include <stm32f0xx_hal.h>
#include <stm32f0xx_hal_uart.h>
#include <bb_global.h>
#include <layer2.h>

#define DOUBLE_BUFFER_SIZE 250

/*!
 * This file defines the layer 1-2 sublayer in charge of
 *  - frame Bigfoot 21 coding/decoding (encodes 0x7e by 0x4042 and 0x40 by 0x4041)
 *  - frame delimitation (opens with 0x7e)
 */

struct L2_frame_header;

void init_sublayer12();
void process_byte(uint8_t uart_idx, uint8_t byte);
uint8_t enqueue(circular_buffer *c, uint8_t v);
uint8_t dequeue(circular_buffer *c, uint8_t *v);
void send_frame(struct L2_frame_header *frame, uint8_t uart_idx);
void buffer_outgoing_byte(uint8_t byte, uint8_t uart_idx);

#endif // _SUBLAYER_12_
