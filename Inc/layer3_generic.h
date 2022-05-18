#ifndef _LAYER3_GENERIC_H_
#define _LAYER3_GENERIC_H_

#include <stm32f0xx_hal.h>
#include <bb_global.h>
#include <bbpp.h>
#include <soft_id.h>

/*!
 * \brief enum L3_packet_type defines the type of packet encapsulated in a frame
 * When an ACK frame is sent, you must use the same packet type so it is properly processed
 */
typedef enum _packed {
	L3_NONE, /* Default initialized value */
	L3_NORMAL, /* Standard source/destination packet */
	L3_STP, /* Spanning tree packet */
	L3_BBPP, /* Blinky block Batch Programming Protocol */
	L3_DEVICE, /* Device commands */
	L3_SOFT_ID, /* Soft ID configuration */
} L3_packet_type;

struct _L3_functions;
typedef uint8_t stp_command;

typedef union {
	#ifdef BBPP
		BBPP_packet bbpp_packet;
	#endif
	#ifdef STP
		stp_command stp_packet;
	#endif
	#ifdef SOFT_ID
		soft_id_packet softid_packet;
	#endif
	#ifdef L3_STANDARD
		uint8_t normal_packet[64];
	#endif
	#ifdef DEVICE_COMMANDS
		uint8_t device_cmd_packet[65];
	#endif
} all_types_packet;

/*!
 * \struct L3_header defines a layer 3 packet header. It is defined by the index of the actuel packet in the actuel L3
 * implementation packets buffer, along with a pointer to the list of L3 functions function pointers.
 * my_idx is the generic header index in the generic headers buffer. It is useful to release itself.
 */
typedef struct _L3_packet {
	uint8_t io_port; ///< Port on which the packet is received/sent
	uint8_t my_idx; ///< Index of packet in packet list (useful for clearing)
	uint8_t packet_content[sizeof(all_types_packet)]; ///< Packet binary content
	struct _L3_functions *functions;
} L3_packet;

typedef struct _L3_functions {
	uint8_t (*process_packet)(L3_packet *p);
	void (*ack_handler)(L3_packet *p);
	void (*unack_handler)(L3_packet *p);
} L3_functions;

#include <stp.h>

void init_L3_packets();
void release_L3_packet(L3_packet *header);
L3_packet *get_free_L3_packet(L3_packet_type pkt_type);

uint8_t process_standard_packet(L3_packet *packet);

/*
 * END OF TESTING
 */

#endif //_LAYER3_GENERIC_H_
