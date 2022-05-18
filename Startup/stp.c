/*
 * stp.c
 *
 *  Created on: 27 mai 2020
 *      Author: flassabe
 */

#include <stp.h>

#ifdef STP
#include <layer2.h>
#include <hwLED.h>
#include <serial.h>

int8_t my_parent_uart = -1; // Defines the parent by its UART index, -1 if no parent is defined
uint8_t my_children_uart = 0; 	// Defines children by flagging bits 0 to 5 related to UART indices
							// Bit i to 0 means that UART #i neighbor is not my child
							// Bit i to 1 means that UART #i neighbor is my child

L3_functions stp_functions = {
		.process_packet = &process_STP_packet,
		.ack_handler = NULL,
		.unack_handler = NULL,
};

uint8_t process_STP_packet(L3_packet *p) {
	stp_command c = (p->packet_content[0] & 0xc0);
	if (c == 0x40) { // Parent request
		L3_packet *parent_response = get_free_L3_packet(L3_STP);
		if (!parent_response) return 1; // Error, cannot respond
		parent_response->io_port = p->io_port;
		if (my_parent_uart == -1 || my_parent_uart == p->io_port) { // No parent or same parent: accept
			my_parent_uart = p->io_port;
			parent_response->packet_content[0] = 0x80;
			for (uint8_t i=0; i<NB_SERIAL_PORT; ++i) { // Propagate to my other UART
				if (i != my_parent_uart && is_connected(i)) {
					L3_packet *child_request = get_free_L3_packet(L3_STP);
					if (child_request) {
						child_request->packet_content[0] = 0x40;
						child_request->io_port = i;
						send_layer3_packet(child_request, L3_STP, 1, 1);
					}
				}
			}
		} else {
			parent_response->packet_content[0] = 0xc0;
		}
		send_layer3_packet(parent_response, L3_STP, 1, 1);
	} else if (c == 0x80) { // Child accept
		my_children_uart |= (1 << p->io_port);
	} else if (c == 0) { // Reset STP
		if (my_parent_uart != -1) { // Don't flood if already clear, would lead to broadcast packet storm
			for (uint8_t i=0; i<NB_SERIAL_PORT; ++i) {
				if (i != my_parent_uart && is_connected(i)) {
					L3_packet *clear_request = get_free_L3_packet(L3_STP);
					if (clear_request) {
						clear_request->packet_content[0] = 0x00;
						clear_request->io_port = i;
						send_layer3_packet(clear_request, L3_STP, 1, 1);
					}
				}
			}
			my_parent_uart = -1;
			my_children_uart = 0;
		}
	} // else: child reject, do nothing
	return 0;
}

void copy_packet_to_children(L3_packet *p, L3_packet_type ptype, uint16_t size) {
	if (my_children_uart != 0) {
		for (uint8_t i=0; i<NB_SERIAL_PORT; ++i) {
			if ((my_children_uart & (1<<i)) != 0) {
				L3_packet *packet = get_free_L3_packet(ptype);
				if (packet) {
					packet->io_port = i;
					packet->functions = NULL;
					for (uint8_t j=0; j<size; ++j) {
						packet->packet_content[j] = p->packet_content[j];
					}
					send_layer3_packet(packet, ptype, size, 1);
				}
			}
		}
	}
}

uint8_t is_child(uint8_t uart_index) {
	return (my_children_uart & (1 << uart_index)) != 0;
}

#endif // STP
