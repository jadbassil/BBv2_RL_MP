/*
 * layer2.c
 *
 *  Created on: Apr 23, 2020
 *      Author: flassabe
 */

#include <layer2.h>
#include <bb_global.h>
#include <hwLED.h>
#include <speaker.h>

L2_frame_header L2_headers_buffer[NB_BUFFER_MAX];
BUFFER_MASK_TYPE used_headers = 0; // Mask with ones for used buffers, zero for available buffers

void init_frames_buffer() {
	for (uint8_t i=0; i<NB_BUFFER_MAX; ++i) {
		L2_headers_buffer[i].header = 0;
		L2_headers_buffer[i].size_lsb = 0;
//		L2_headers_buffer[i].L3_pkt_type = L3_UNKNOWN;
		L2_headers_buffer[i].L3_pkt_type = 0;
		L2_headers_buffer[i].frame_id = 0;
		L2_headers_buffer[i].my_id = i;
	}
}

L2_frame_header *get_free_L2_header() {
	for (uint8_t i=0; i<NB_BUFFER_MAX; ++i) {
		if (IS_BUFFER_FREE(used_headers, i)) {
			MARK_BUFFER_USED(used_headers, i);
			L2_headers_buffer[i].header = 0;
			L2_headers_buffer[i].tx_retries = 0;
			L2_headers_buffer[i].tx_timestamp = 0; // Default value for received and/or not reemitted frames
			return &L2_headers_buffer[i];
		}
	}
	return NULL;
}

/*!
 * \function process_frame processes a frame validated by the MAC sublayer21
 * It returns therefore dropping the packet on any error encountered.
 * Only a completely good frame will be decapsulated and forwarded to network layer.
 *
 * \param c a pointer to the circular buffer containing the frame bytes
 * \param uart_idx is the index of the receiving frame UART interface (from 0 to 5)
 */
void process_frame(circular_buffer *c, uint8_t uart_idx) {
	uint8_t byte;
	uint16_t len = 0;
	uint16_t checksum = 0;
	uint16_t pkt_checksum = 0;
	L2_frame_header *frame = get_free_L2_header();
	if (frame != NULL) { // A frame header is available
		//setHWLED(50, 50, 0);
		uint8_t *hdr = (uint8_t *) frame;
		for (uint8_t i=0; i<4; ++i) {
			if (dequeue(c, &hdr[i]) == 0) { // Header byte not present
				UNMARK_BUFFER_USED(used_headers, frame->my_id);
				return;
			}
			compute_checksum(&checksum, hdr[i]);
		}

		if (IS_L2_TYPE_PPP_ACK(frame->header)) { // This is an ack
			if (frame->size_lsb != 8) {
				UNMARK_BUFFER_USED(used_headers, frame->my_id);
				return; // Error
			}
			uint8_t chksum_byte;
			for (uint8_t i=0; i<2; ++i) {
				if (dequeue(c, &chksum_byte) == 0) { // Header byte not present
					UNMARK_BUFFER_USED(used_headers, frame->my_id);
					return;
				}
				compute_checksum(&checksum, chksum_byte);
			}
			if (dequeue(c, &chksum_byte) == 0) { // Header byte not present
				UNMARK_BUFFER_USED(used_headers, frame->my_id);
				return;
			}
			pkt_checksum = chksum_byte * 256;
			if (dequeue(c, &chksum_byte) == 0) { // Header byte not present
				UNMARK_BUFFER_USED(used_headers, frame->my_id);
				return;
			}
			pkt_checksum += chksum_byte;
			if (pkt_checksum == checksum) {
				release_L3_packet(L2_headers_buffer[frame->frame_id].L3_pkt_p);
				UNMARK_BUFFER_USED(used_headers, frame->frame_id);
			}
			UNMARK_BUFFER_USED(used_headers, frame->my_id); // Also free current frame
			return; // Exit, ACK has been processed
		}

		len = (frame->header & 0x03) * 256 + frame->size_lsb;
		frame->L3_pkt_p = get_free_L3_packet(frame->L3_pkt_type);
		if (frame->L3_pkt_p == NULL) { // No resource to process message
//			setHWLED(50, 50, 0);
			UNMARK_BUFFER_USED(used_headers, frame->my_id); // clear my frame
			for (uint16_t i=4; i<len; ++i) { // Empty remaining frame bytes
				if (dequeue(c, &byte) == 0) { // No byte to read: stop
					return;
				}
			}
			return;
		}

		// Now we read as many bytes as the remaining size-2 (CRC bytes)
		for (uint16_t i=4; i<len-2; ++i) {
			if (dequeue(c, &frame->L3_pkt_p->packet_content[i-4]) == 0) { // No byte to read: frame error
				release_L3_packet(frame->L3_pkt_p);
				UNMARK_BUFFER_USED(used_headers, frame->my_id);
				return;
			}
			compute_checksum(&checksum, frame->L3_pkt_p->packet_content[i-4]);
		}
		// Extract checksum
		if (dequeue(c, &byte) == 0) { // No byte to read: frame error
			release_L3_packet(frame->L3_pkt_p);
//			setHWLED(0, 50, 50);
			UNMARK_BUFFER_USED(used_headers, frame->my_id);
			return;
		}
		pkt_checksum = byte<<8;
		if (dequeue(c, &byte) == 0) { // No byte to read: frame error
			release_L3_packet(frame->L3_pkt_p);
//			setHWLED(50, 50, 50);
			UNMARK_BUFFER_USED(used_headers, frame->my_id);
			return;
		}
		pkt_checksum += byte;
		if (pkt_checksum != checksum) {
			UNMARK_BUFFER_USED(used_headers, frame->my_id);
			release_L3_packet(frame->L3_pkt_p);
//			setHWLED(50, 0, 0);
			return;
		}
//		setHWLED(0, 50, 0);

		L3_packet *l3_pkt = frame->L3_pkt_p;
		if (IS_L2_PPP_ACK_REQUIRED(frame->header)) { // ACK is required by frame source
			frame->header = 0; // Put size Msb to zero
			frame->size_lsb = 8; // frame header (4) + source CRC (2) + ACK CRC (2)
			L3_packet packet = {
					.functions = NULL,
					.io_port = uart_idx,
					.my_idx = 255,
			};
			frame->L3_pkt_p = &packet;
			uint8_t *ptr = (uint8_t *) &pkt_checksum;
			packet.packet_content[0] = ptr[1];
			packet.packet_content[1] = ptr[0];
			frame->tx_retries = 1;
			send_frame(frame, uart_idx);
		}
		// Everything went well, fill in remaining fields and let higher layer process its packet
		UNMARK_BUFFER_USED(used_headers, frame->my_id); // Release frame before layer 3 processing
		l3_pkt->io_port = uart_idx;
		l3_pkt->functions->process_packet(l3_pkt);
		release_L3_packet(l3_pkt);
//		setHWLED(0, 50, 0);
	}
}

void send_delayed_layer3_packet(L3_packet *p, L3_packet_type pkt_type, uint16_t size, uint16_t delay_ms, uint8_t with_ack) {
	if (p == NULL) return;
	L2_frame_header *frame = get_free_L2_header();
	if (frame != NULL) {
		size += 6;
		frame->tx_timestamp = HAL_GetTick() + (uint32_t) delay_ms;
		if (with_ack == 1) {
			SET_L2_PPP_ACK_REQUIRED(frame->header);
			frame->tx_retries = NB_TX_RETRIES;
		} else
			frame->tx_retries = 1;
		SET_L2_TYPE_REQUEST(frame->header);
		frame->header |= (size >> 8) & 0x03;
		frame->size_lsb = (size & 0xff);
		frame->L3_pkt_type = pkt_type;
		frame->frame_id = frame->my_id;
		frame->L3_pkt_p = p;
//		send_frame(frame, frame->L3_pkt_p->io_port);
	} else {
		release_L3_packet(frame->L3_pkt_p);
	}
}

void send_layer3_packet(L3_packet *p, L3_packet_type pkt_type, uint16_t size, uint8_t with_ack) {
	if (p == NULL) return;
	L2_frame_header *frame = get_free_L2_header();
	if (frame != NULL) {
		size += 6;
		if (with_ack == 1) {
			SET_L2_PPP_ACK_REQUIRED(frame->header);
			frame->tx_retries = NB_TX_RETRIES;
		} else
			frame->tx_retries = 1;
		SET_L2_TYPE_REQUEST(frame->header);
		frame->header |= (size >> 8) & 0x03;
		frame->size_lsb = (size & 0xff);
		frame->L3_pkt_type = pkt_type;
		frame->frame_id = frame->my_id;
		frame->L3_pkt_p = p;
		send_frame(frame, frame->L3_pkt_p->io_port);
	} else {
		release_L3_packet(frame->L3_pkt_p);
	}
}

void check_pending_frames() {
	extern uint8_t has_frames_ready[NB_SERIAL_PORT];
	extern circular_buffer inbuffer[NB_SERIAL_PORT];

	for (uint8_t i=0; i<NB_SERIAL_PORT; ++i) {
		if (has_frames_ready[i] > 0) {
			--has_frames_ready[i];
			process_frame(&inbuffer[i], i);
			continue;
		}
	}
}

void check_reemitted_frames() {
	uint32_t now = HAL_GetTick();
	for (uint8_t i=0; i<NB_BUFFER_MAX; ++i) {
		if (!IS_BUFFER_FREE(used_headers, i)) {
			if (L2_headers_buffer[i].tx_retries > 0) { // Frames not ACK'ed, and delayed frames
				if ((now > L2_headers_buffer[i].tx_timestamp) && L2_headers_buffer[i].L3_pkt_p != NULL) {
					send_frame(&L2_headers_buffer[i], L2_headers_buffer[i].L3_pkt_p->io_port);
				}
			} else { // Frames not ACK'ed whose retries have all been tried: clear
				if (now > L2_headers_buffer[i].tx_timestamp) {
					L3_packet *the_packet = L2_headers_buffer[i].L3_pkt_p;
					L2_headers_buffer[i].L3_pkt_p = NULL;
					UNMARK_BUFFER_USED(used_headers, i);
					if (the_packet != NULL) {
						if (the_packet->functions->unack_handler != NULL)
							the_packet->functions->unack_handler(the_packet);
						release_L3_packet(the_packet);
					}
				}
			}
		}
	}
}

void compute_checksum(uint16_t * checksumPtr, uint8_t data) {
	uint16_t checksum=*checksumPtr;
	checksum = (checksum >> 1) + ((checksum & 1) << 15);
	checksum += data;
	*checksumPtr=checksum;
}
