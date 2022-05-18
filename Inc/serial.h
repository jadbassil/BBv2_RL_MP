#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <bb_global.h>

#define BUFFER_SIZE 512
#define BUFFER_MASK (BUFFER_SIZE-1)

#define HELLO_PROTOCOL_HELLO_PACKET_DELAY_MS 100
#define HELLO_PROTOCOL_HELLO_PACKET_TIMEOUT_MS 250

typedef struct {
	uint8_t write_buffer[BUFFER_SIZE];	///< Buffer for outgoing operations
//	uint8_t read_buffer[BUFFER_SIZE];	///< Buffer for reception
//	uint16_t read_rpt;					///< Consumer pointer to read buffer
//	uint16_t read_wpt;					///< Producer pointer to read buffer
	uint16_t write_rpt;					///< Consumer pointer to write buffer
	uint16_t write_wpt;					///< Producer pointer to write buffer
	uint8_t tx_status;					///< Transmission status: 0 not transmitting, 1 transmitting
} uart_comm_information;

void init_serial_communications();
void receive_byte(uint8_t uart_idx, uint8_t byte);
void transmit_byte(uint8_t uart_idx, uint8_t byte);
uint8_t serial_index_from_handle(UART_HandleTypeDef *huart);
UART_HandleTypeDef *serial_handle_from_index(uint8_t index);
uart_comm_information *get_uart_comm(uint8_t uart_idx);

void init_hello_protocol();
uint8_t is_connected(uint8_t uart_index);
void check_last_sent();

#endif // _SERIAL_H_
