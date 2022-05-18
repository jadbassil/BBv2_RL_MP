/*
 * BB.c
 *
 *  Created on: 10 juil. 2020
 *      Author: flassabe
 */

#include <BB.h>
#include <flash_storage.h>
#include <serial.h>
#include <layer3_generic.h>
#include <layer2.h>

extern configuration *my_configuration;
extern uint8_t is_initialized;

uint32_t getId() {
	if (my_configuration)
		return my_configuration->soft_id;
	return 0;
}

void sendMessage(uint8_t port, uint8_t data[], uint16_t size, uint8_t has_ack) {
	if (port > 5) return;
	L3_packet *packet = get_free_L3_packet(L3_NORMAL);
	if (packet) {
		packet->io_port = port;
		for (uint16_t i=0; i<size; ++i)
			packet->packet_content[i] = data[i];
		if (is_initialized)
			send_layer3_packet(packet, L3_NORMAL, size, has_ack);
		else
			send_delayed_layer3_packet(packet, L3_NORMAL, size, 1000, has_ack);
	}
}
