/*
 * layer3_generic.c
 *
 *  Created on: 30 avr. 2020
 *      Author: flassabe
 */

#include <layer3_generic.h>
#include <stm32f0xx_hal.h>
#include <hwLED.h>

#define NB_L3_PACKETS 32
#define ALL_ONES_MASK 0xffffffff

#ifdef BBPP
	extern L3_functions bbpp_functions;
#endif // BBPP
#ifdef STP
	extern L3_functions stp_functions;
#endif // STP
#ifdef DEVICE_COMMANDS
	extern L3_functions device_commands_functions;
#endif
#ifdef SOFT_ID
	extern L3_functions soft_id_functions;
#endif

L3_packet l3_generic_packets[NB_L3_PACKETS];
uint32_t l3_generic_mask = 0;

// Internal
L3_packet *get_free_packet() {
	for (uint8_t i=0; i<NB_L3_PACKETS; ++i) {
		if (((l3_generic_mask>>i) & 0x1) == 0) {
			l3_generic_mask |= (1<<i);
			return &l3_generic_packets[i];
		}
	}
	return NULL;
}

void init_L3_packets() {
	for (uint8_t i=0; i<NB_L3_PACKETS; ++i) {
		l3_generic_packets[i].my_idx = i;
	}
}

void release_L3_packet(L3_packet *packet) {
	if (packet->my_idx < NB_L3_PACKETS)
		l3_generic_mask &= ALL_ONES_MASK ^ (1<<packet->my_idx);
}

__weak uint8_t process_standard_packet(L3_packet *packet) {
	return 0;
}

__weak void standard_ack_handler(L3_packet *p) {
}

__weak void standard_unack_handler(L3_packet *p) {
}

L3_functions l3_standard_functions = {
		.process_packet = &process_standard_packet,
		.ack_handler = &standard_ack_handler,
		.unack_handler = &standard_unack_handler,
};

L3_packet *get_free_L3_packet(L3_packet_type pkt_type) {
	L3_packet *packet = get_free_packet();
	if (packet == NULL)
		return NULL;

	#ifdef BBPP
		if (pkt_type == L3_BBPP) {
			packet->functions = &bbpp_functions;
			return packet;
		}
	#endif // BBPP

	#ifdef DEVICE_COMMANDS
		if (pkt_type == L3_DEVICE) {
			packet->functions = &device_commands_functions;
			return packet;
		}
	#endif // DEVICE_COMMANDS

	#ifdef SOFT_ID
		if (pkt_type == L3_SOFT_ID) {
			packet->functions = &soft_id_functions;
			return packet;
		}
	#endif

	#ifdef L3_STANDARD
		if (pkt_type == L3_NORMAL) {
			packet->functions = &l3_standard_functions;
			return packet;
		}
	#endif // L3_STANDARD

	#ifdef STP
		if (pkt_type == L3_STP) {
			packet->functions = &stp_functions;
			return packet;
		}
	#endif // STP

	release_L3_packet(packet);
	return NULL; // L3 protocol not present/unknown
}

