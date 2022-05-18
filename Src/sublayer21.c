/*
 * sublayer21.c
 *
 *  Created on: 24 avr. 2020
 *      Author: flassabe
 */

#include <sublayer12.h>
#include <layer2.h>
#include <serial.h>
#include <hwLED.h>

circular_buffer inbuffer[NB_SERIAL_PORT];
uint8_t pkt_reading_state[NB_SERIAL_PORT]; 	// 0x00 for initial state
											// 0x40 for frame started (serial port received a '~')

uint8_t previous_byte_received[NB_SERIAL_PORT]; // Last byte if BF21 prefix (0x40), 0x0 else
uint8_t has_frames_ready[NB_SERIAL_PORT] = {0, 0, 0, 0, 0, 0};

/*!
 * \brief process_byte processes a byte received on a UART interface
 * It decodes BF21 encoded frames and checks the frame delimiters.
 * When a frame is fully decoded, it is sent to the layer2.
 * \param huart the pointer to the UART handle that received the byte
 */
void process_byte(uint8_t uart_idx, uint8_t byte) {
	uint8_t data=byte;
	if (uart_idx > 5) return;
//	setHWLED(0, 0, 50);

	if (data == 0x40) {
		previous_byte_received[uart_idx] = 0x40;
		return;
	}

	if (data == '~') { // Start of frame, byte not stored in circular buffer
		if (pkt_reading_state[uart_idx] != 0x0) { // Previous frame incomplete, cancel it
			previous_byte_received[uart_idx] = 0;
			BUFFER_REINIT(&inbuffer[uart_idx]);
//			setHWLED(0, 50, 50);
		}
		pkt_reading_state[uart_idx] = 0x40;  // Set current state to "reading frame"
	} else if (data == '.' && pkt_reading_state[uart_idx] == 0x40) {
		// End of frame marker when reading frame: finish and transfer to higher layer
//		setHWLED(0, 50, 0);
		++has_frames_ready[uart_idx];
		pkt_reading_state[uart_idx] = 0x0;
	} else if (pkt_reading_state[uart_idx] == 0x40) { // We check if current byte must be decoded
		if (previous_byte_received[uart_idx] == 0x40) {
			// If BF21 encoded character: decode it
			if (data == 0x41)
				data = 0x40;
			else if (data == 0x42)
				data = 0x7e;
			else if (data == 0x43)
				data = '.';
			else if (data == 0x44)
				data = 'H';
			previous_byte_received[uart_idx] = 0;
		}
		enqueue(&inbuffer[uart_idx], data);
	} // else: received something outside delimiters: do nothing
}

/*!
 * \brief init_sublayer12 inits all buffers of the sublayer12
 */
void init_sublayer12() {
	for (uint8_t i=0; i<NB_SERIAL_PORT; ++i) {
		pkt_reading_state[i] = 0x0;
		previous_byte_received[i] = 0x0;
		BUFFER_INIT(&inbuffer[i]);
	}
}

/*!
 * \brief enqueue adds a byte to a circular buffer
 * \param c pointer to the target circular buffer
 * \param v the byte to enqueue
 * \return 0 (error) if the buffer was full, 1 (success) else
 */
uint8_t enqueue(circular_buffer *c, uint8_t v) {
	if (BUFFER_FULL(c)) {
		return 0;
	}
	c->content[c->pWR] = v;
	c->pWR = (c->pWR + 1) % CIRCULAR_LENGTH;
	return 1;
}

/*!
 * \brief dequeue fetches a byte from a circular buffer
 * \param c pointer to the source circular buffer
 * \param v a pointer to the byte fetched
 * \return 0 (error) if the buffer was empty, 1 (success) else
 */
uint8_t dequeue(circular_buffer *c, uint8_t *v) {
	if (BUFFER_EMPTY(c)) {
		return 0;
	}
	*v = c->content[c->pRD];
	c->pRD = (c->pRD + 1) % CIRCULAR_LENGTH;
	return 1;
}

/*!
 * \brief send_frame sends a frame to a UART interface
 */
void send_frame(struct L2_frame_header *frame, uint8_t uart_idx) {
//	if (frame->tx_timestamp > HAL_GetTick()) return;
	if (uart_idx > 5) return;
	transmit_byte(uart_idx, '~');
	uint16_t checksum = 0;
	uint8_t *buffer_ptr = (uint8_t *) frame;
	for (uint8_t i=0; i<4; ++i) {
		buffer_outgoing_byte(buffer_ptr[i], uart_idx);
		compute_checksum(&checksum, buffer_ptr[i]);
	}
	uint16_t size = (frame->header & 0x03) * 256 + frame->size_lsb;
	buffer_ptr = frame->L3_pkt_p->packet_content;
	for (uint8_t i=0; i<size-6; ++i) {
		buffer_outgoing_byte(buffer_ptr[i], uart_idx);
		compute_checksum(&checksum, buffer_ptr[i]);
	}
	buffer_ptr = (uint8_t *) &checksum;
	buffer_outgoing_byte(buffer_ptr[1], uart_idx);
	buffer_outgoing_byte(buffer_ptr[0], uart_idx);
	transmit_byte(uart_idx, '.');
	if (IS_L2_PPP_ACK_DISABLED(frame->header)) {
		extern BUFFER_MASK_TYPE used_headers;
		release_L3_packet(frame->L3_pkt_p);
		UNMARK_BUFFER_USED(used_headers, frame->my_id);
	} else {
		frame->tx_retries--;
		frame->tx_timestamp = HAL_GetTick() + FRAME_ACK_TIMEOUT_MS;
	}
}

/*!
 * \brief buffer_outgoing_byte adds a byte to the outgoing buffer after BF21 encoding
 * If the target outgoing buffer is full, it segments the frame, sends the current
 * buffer and returns the index of a new buffer to which the frame will be resumed
 * \param byte the byte to add to the buffer
 * \param buf the buffer pointer
 * \return the buffer pointer to continue enqueuing bytes
 */
void buffer_outgoing_byte(uint8_t byte, uint8_t uart_idx) {
	if (uart_idx >= NB_SERIAL_PORT) return;

	if (byte == '@') {
		transmit_byte(uart_idx, '@'); // 0x40
		transmit_byte(uart_idx, 'A'); // 0x41
	} else if (byte == '~') {
		transmit_byte(uart_idx, '@'); // 0x40
		transmit_byte(uart_idx, 'B'); // 0x42
	} else if (byte == '.') {
		transmit_byte(uart_idx, '@'); // 0x40
		transmit_byte(uart_idx, 'C'); // 0x43
	} else if (byte == 'H') {
		transmit_byte(uart_idx, '@'); // 0x40
		transmit_byte(uart_idx, 'D'); // 0x44
	} else
		transmit_byte(uart_idx, byte);
}
