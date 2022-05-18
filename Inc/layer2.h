#ifndef _LAYER2_H_
#define _LAYER2_H_

#include <stm32f0xx_hal.h>
#include <layer3_generic.h>
#include <sublayer12.h>

typedef struct L2_frame_header {
	uint8_t header; // Header:  bit 7: frame type (0 -> frame, 1 -> PPP ack)
					//			bit 6: is PPP ack requested (0 -> no, 1 -> yes)
					//			bit 1: size field 1st MSb
					//			bit 0: size field 2nd MSb
	uint8_t size_lsb; // Size MSb are in header byte above
	uint8_t L3_pkt_type; // based on enum L3_packet_type
	uint8_t frame_id; // Unique frame id for ack return matching,
					  // must be unique > 0 if PPP ack requested, 0 otherwise
					  // Uses the frame index in frame buffer list
	uint8_t my_id; // Local id of the frame (used for cleanup).
	uint8_t tx_retries; // Number of remaining transmissions before dropping frame
	uint32_t tx_timestamp; // Date the frame was sent
	L3_packet *L3_pkt_p; // Pointer to network layer packet
} L2_frame_header;

#define SET_L2_TYPE_REQUEST(header) (header |= 0x80)
#define SET_L2_TYPE_PPP_ACK(header) (header &= 0x7f)
#define SET_L2_PPP_ACK_REQUIRED(header) (header |= 0x40)
#define SET_L2_PPP_ACK_DISABLED(header) (header &= 0xbf)
#define IS_L2_TYPE_REQUEST(header) ((header & 0x80) == 0x80)
#define IS_L2_TYPE_PPP_ACK(header) ((header & 0x80) == 0x00)
#define IS_L2_PPP_ACK_REQUIRED(header) ((header & 0x40) == 0x40)
#define IS_L2_PPP_ACK_DISABLED(header) ((header & 0x40) == 0x00)

void init_frames_buffer();
void process_frame(circular_buffer *c, uint8_t uart_idx);
void compute_checksum(uint16_t * checksumPtr, uint8_t data);
void send_delayed_layer3_packet(L3_packet *p, L3_packet_type pkt_type, uint16_t size, uint16_t delay_ms, uint8_t with_ack);
void send_layer3_packet(L3_packet *p, L3_packet_type pkt_type, uint16_t size, uint8_t with_ack);
L2_frame_header *get_free_L2_header();
void check_pending_frames();
void check_reemitted_frames();

#endif //_LAYER2_H_
