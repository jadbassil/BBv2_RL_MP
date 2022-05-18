/*
 * device_commands.c
 *
 *  Created on: 18 juin 2020
 *      Author: flassabe
 */
#include <device_commands.h>
#include <hwLED.h>

#ifdef DEVICE_COMMANDS

#include <flash_storage.h>
#include <layer2.h>

L3_functions device_commands_functions = {
		.process_packet = &process_device_command_packet,
		.ack_handler = NULL,
		.unack_handler = NULL,
};

extern uint32_t time_to_jump;
extern uint32_t address_to_jump;

uint8_t process_device_command_packet(L3_packet *p) {
	if (!p) return -1;
	uint8_t cmd = p->packet_content[0];
	if (cmd == DEVICE_COMMAND_TYPE_LED) {
		setHWLED(p->packet_content[1], p->packet_content[2], p->packet_content[3]);
		copy_packet_to_children(p, L3_DEVICE, 4);
	} else if (cmd == DEVICE_COMMAND_TYPE_START_APP) {
		copy_packet_to_children(p, L3_DEVICE, 5);
		address_to_jump = (p->packet_content[1]<<24) + (p->packet_content[2]<<16) + (p->packet_content[3]<<8) + p->packet_content[4];
		time_to_jump = HAL_GetTick() + 1000;
		copy_packet_to_children(p, L3_DEVICE, 4);
	} else if (cmd == DEVICE_COMMAND_TYPE_APP_AUTOSTART) { // One byte argument: 0 no autostart, 1 autostart
		set_parameter(1, (uint32_t) p->packet_content[1]);
		copy_packet_to_children(p, L3_DEVICE, 4);
	} else if (cmd == DEVICE_COMMAND_TYPE_APP_AUTOSTART_DELAY) { // 1 byte argument: seconds count before autostart
		uint32_t time_value = (uint32_t) p->packet_content[1];
		if (time_value > 10) time_value = 10; // No more than 10 seconds
		set_parameter(2, time_value);
		copy_packet_to_children(p, L3_DEVICE, 4);
	} else if (cmd == DEVICE_COMMAND_TYPE_SEND_CONFIGURATION) {
		extern configuration *my_configuration;
		if (my_configuration) {
			L3_packet *packet = get_free_L3_packet(L3_DEVICE);
			if (packet) {
				packet->io_port = p->io_port;
				packet->functions = NULL;
				packet->packet_content[0] = DEVICE_COMMAND_TYPE_SEND_CONFIGURATION_RESPONSE;
				uint32_t *ptr = (uint32_t *) my_configuration;
				for (uint8_t i=0; i<16; ++i) {
					packet->packet_content[4*i+1] = ((ptr[i]>>24) & 0xff);
					packet->packet_content[4*i+2] = ((ptr[i]>>16) & 0xff);
					packet->packet_content[4*i+3] = ((ptr[i]>>8) & 0xff);
					packet->packet_content[4*i+4] = (ptr[i] & 0xff);
				}
				send_layer3_packet(packet, L3_DEVICE, 65, 1);
			}
		}
	} else if (cmd == DEVICE_COMMAND_TYPE_APP_COMMIT_CONFIGURATION) {
		save_configuration();
		copy_packet_to_children(p, L3_DEVICE, 4);
	} else if (cmd == DEVICE_COMMAND_TYPE_REINIT_CONFIGURATION) {
		reinit_configuration();
		copy_packet_to_children(p, L3_DEVICE, 4);
	} else if (cmd > 2 && cmd < 32) { // Within range of parameters
		if (cmd != 4) { // Cannot erase application address parameter, since it is inferred from HEX information
			uint32_t param_val = (p->packet_content[1]<<24) + (p->packet_content[2]<<16) + (p->packet_content[3]<<8) + p->packet_content[4];
			set_parameter(cmd, param_val);
		}
	} else if (cmd == DEVICE_COMMAND_TYPE_REBOOT) {
		copy_packet_to_children(p, L3_DEVICE, 1);
		extern uint8_t must_reboot;
		extern uint32_t reboot_time;
		must_reboot = 1;
		reboot_time = HAL_GetTick() + 500;
	}
	return 0;
}

#endif // DEVICE_COMMANDS
