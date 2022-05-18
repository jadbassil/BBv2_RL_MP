/*
 * soft_id.c
 *
 *  Created on: 29 juin 2020
 *      Author: flassabe
 */
#include <soft_id.h>
#include <layer3_generic.h>
#include <layer2.h>
#include <flash_storage.h>

L3_functions soft_id_functions = {
		.process_packet = &soft_id_process_packet,
		.ack_handler = NULL,
		.unack_handler = NULL,
};

uint8_t responding_children = 0;

uint32_t max_id = 0;
uint8_t current_uart = 0;

uint8_t process_children_count_request(L3_packet *p);
uint8_t process_children_count_response(L3_packet *p);
uint8_t process_set_soft_id_request(L3_packet *p);
uint8_t process_set_soft_id_response(L3_packet *p);

uint8_t soft_id_process_packet(L3_packet *p) {
#ifdef SOFT_ID
	uint8_t command = p->packet_content[0];
	if (command == SOFT_ID_PROTO_COUNT_CHILDREN_REQUEST)
		process_children_count_request(p);
	else if (command == SOFT_ID_PROTO_COUNT_CHILDREN_RESPONSE)
		process_children_count_response(p);
	else if (command == SOFT_ID_PROTO_SET_SOFT_ID_REQUEST)
		process_set_soft_id_request(p);
	else if (command == SOFT_ID_PROTO_SET_SOFT_ID_RESPONSE)
		process_set_soft_id_response(p);
#endif
	return 0;
}

/*!
 * \brief handles a request for children recursive counting
 * if no children, sends back one (itself) to parent, else propagate packet to all children
 * */
uint8_t process_children_count_request(L3_packet *p) {
	return 0;
}

uint8_t process_children_count_response(L3_packet *p) {
	return 0;
}

uint8_t process_set_soft_id_request(L3_packet *p) {
	extern uint8_t my_children_uart;
	extern int8_t my_parent_uart;
	uint32_t first_id = (p->packet_content[1]<<24)+(p->packet_content[2]<<16)+(p->packet_content[3]<<8)+p->packet_content[4];
	set_parameter(3, first_id);
	save_configuration();
	max_id = first_id;
	first_id = max_id + 1;
	if (my_children_uart != 0) {
		for (current_uart=0; current_uart<NB_SERIAL_PORT; ++current_uart) {
			if (is_child(current_uart)) {
				L3_packet *packet = get_free_L3_packet(L3_SOFT_ID);
				if (packet) {
					packet->io_port = current_uart;
					packet->functions = NULL;
					packet->packet_content[0] = SOFT_ID_PROTO_SET_SOFT_ID_REQUEST;
					packet->packet_content[1] = ((first_id>>24) & 0xff);
					packet->packet_content[2] = ((first_id>>16) & 0xff);
					packet->packet_content[3] = ((first_id>>8) & 0xff);
					packet->packet_content[4] = (first_id & 0xff);
					send_layer3_packet(packet, L3_SOFT_ID, 5, 1);
				}
				break;
			}
		}
	} else { // BB has no child
		L3_packet *packet = get_free_L3_packet(L3_SOFT_ID);
		if (packet) {
			packet->io_port = my_parent_uart;
			packet->functions = NULL;
			packet->packet_content[0] = SOFT_ID_PROTO_SET_SOFT_ID_RESPONSE;
			packet->packet_content[1] = ((max_id>>24) & 0xff);
			packet->packet_content[2] = ((max_id>>16) & 0xff);
			packet->packet_content[3] = ((max_id>>8) & 0xff);
			packet->packet_content[4] = (max_id & 0xff);
			send_layer3_packet(packet, L3_SOFT_ID, 5, 1);
		}
	}
	return 0;
}

uint8_t process_set_soft_id_response(L3_packet *p) {
	extern int8_t my_parent_uart;
	uint32_t children_max_id = (p->packet_content[1]<<24)+(p->packet_content[2]<<16)+(p->packet_content[3]<<8)+p->packet_content[4];
	if (max_id < children_max_id) max_id = children_max_id;
	uint32_t first_id = max_id + 1;
	++current_uart;
	for (; current_uart<NB_SERIAL_PORT; ++current_uart) {
		if (is_child(current_uart)) {
			L3_packet *packet = get_free_L3_packet(L3_SOFT_ID);
			if (packet) {
				packet->io_port = current_uart;
				packet->functions = NULL;
				packet->packet_content[0] = SOFT_ID_PROTO_SET_SOFT_ID_REQUEST;
				packet->packet_content[1] = ((first_id>>24) & 0xff);
				packet->packet_content[2] = ((first_id>>16) & 0xff);
				packet->packet_content[3] = ((first_id>>8) & 0xff);
				packet->packet_content[4] = (first_id & 0xff);
				send_layer3_packet(packet, L3_SOFT_ID, 5, 1);
			}
			break;
		}
	}
	if (current_uart == NB_SERIAL_PORT) {
		L3_packet *packet = get_free_L3_packet(L3_SOFT_ID);
		if (packet) {
			packet->io_port = my_parent_uart;
			packet->functions = NULL;
			packet->packet_content[0] = SOFT_ID_PROTO_SET_SOFT_ID_RESPONSE;
			packet->packet_content[1] = ((max_id>>24) & 0xff);
			packet->packet_content[2] = ((max_id>>16) & 0xff);
			packet->packet_content[3] = ((max_id>>8) & 0xff);
			packet->packet_content[4] = (max_id & 0xff);
			send_layer3_packet(packet, L3_SOFT_ID, 5, 1);
		}
	}
	return 0;
}
