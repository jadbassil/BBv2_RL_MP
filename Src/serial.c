/*
 * serial.c
 *
 *  Created on: 12 juin 2020
 *      Author: flassabe
 */

#include <serial.h>
#include <sublayer12.h>
#include <stm32f0xx_hal_uart.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

UART_HandleTypeDef *uart_handles[NB_SERIAL_PORT] = {&huart1, &huart2, &huart3, &huart4, &huart5, &huart6};

uart_comm_information uart_buffers[NB_SERIAL_PORT];

#ifdef HELLO
	uint32_t last_byte_received[NB_SERIAL_PORT];
	uint32_t last_byte_sent[NB_SERIAL_PORT];
#endif

void init_serial_communications() {
	for (uint8_t i=0; i<NB_SERIAL_PORT; ++i) {
//		uart_buffers[i].read_rpt = 0;
//		uart_buffers[i].read_wpt = 0;
		uart_buffers[i].write_rpt = 0;
		uart_buffers[i].write_wpt = 0;
		uart_buffers[i].tx_status = 0;
	}
}

void transmit_byte(uint8_t uart_idx, uint8_t byte) {
	if (((uart_buffers[uart_idx].write_wpt+1)&BUFFER_MASK) != uart_buffers[uart_idx].write_rpt) {
		uart_buffers[uart_idx].write_buffer[(uart_buffers[uart_idx].write_wpt++)&BUFFER_MASK] = byte;
		__HAL_UART_ENABLE_IT(uart_handles[uart_idx], UART_IT_TXE);
	}
#ifdef HELLO
	if (byte == '.' || byte == 'H')
		last_byte_sent[uart_idx] = HAL_GetTick();
#endif
}

void receive_byte(uint8_t uart_idx, uint8_t byte) {
#ifdef HELLO
	if (byte == '.')
		last_byte_received[uart_idx] = HAL_GetTick();
	if (byte == 'H') {
		last_byte_received[uart_idx] = HAL_GetTick();
		return;
	}
#endif
	process_byte(uart_idx, byte);
}

/*!
 * \brief serial_index_from_handle gets a UART index from its handle
 * \param huart a pointer to a UART handle
 * \return the corresponding UART index
 */
uint8_t serial_index_from_handle(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) return 0;
	if (huart->Instance == USART2) return 1;
	if (huart->Instance == USART3) return 2;
	if (huart->Instance == USART4) return 3;
	if (huart->Instance == USART5) return 4;
	if (huart->Instance == USART6) return 5;
	return 6; // Means there is an error somewhere
}
/*!
 * \brief serial_handle_from_index gets a pointer to a UART Handle structure from its index
 * \param the UART index
 * \return the corresponding UART Handle pointer
 */
UART_HandleTypeDef *serial_handle_from_index(uint8_t index) {
	if (index < NB_SERIAL_PORT)
		return uart_handles[index];
	return NULL;
}

uart_comm_information *get_uart_comm(uint8_t uart_idx) {
	if (uart_idx >= NB_SERIAL_PORT) return NULL;
	return &uart_buffers[uart_idx];
}

void init_hello_protocol() {
#ifdef HELLO
	uint32_t time_start = HAL_GetTick();
	for (uint8_t i=0; i<NB_SERIAL_PORT; ++i) {
		last_byte_received[i] = time_start;
		last_byte_sent[i] = time_start;
	}
#endif
}

uint8_t is_connected(uint8_t uart_index) {
#ifdef HELLO
	if (HAL_GetTick() > last_byte_received[uart_index] + HELLO_PROTOCOL_HELLO_PACKET_TIMEOUT_MS)
		return 0;
#endif
	return 1; // If no hello protocol, always returns true (1)
}

void check_last_sent() {
#ifdef HELLO
	for (uint8_t i=0; i<NB_SERIAL_PORT; ++i)
		if ((last_byte_sent[i] + HELLO_PROTOCOL_HELLO_PACKET_DELAY_MS) < HAL_GetTick()) {
			transmit_byte(i, 'H');
		}
#endif
}
