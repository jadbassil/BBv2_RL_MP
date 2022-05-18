#ifndef _DEVICE_COMMANDS_H_
#define _DEVICE_COMMANDS_H_

#include <bb_global.h>

#ifdef DEVICE_COMMANDS

#define DEVICE_COMMAND_TYPE_LED 32
#define DEVICE_COMMAND_TYPE_START_APP 33
#define DEVICE_COMMAND_TYPE_REBOOT 34
#define DEVICE_COMMAND_TYPE_APP_AUTOSTART 1
#define DEVICE_COMMAND_TYPE_APP_AUTOSTART_DELAY 2
#define DEVICE_COMMAND_TYPE_REINIT_CONFIGURATION 125
#define DEVICE_COMMAND_TYPE_SEND_CONFIGURATION 126
#define DEVICE_COMMAND_TYPE_SEND_CONFIGURATION_RESPONSE 127
#define DEVICE_COMMAND_TYPE_APP_COMMIT_CONFIGURATION 128

#include <layer3_generic.h>

uint8_t process_device_command_packet(L3_packet *p);

#endif // DEVICE_COMMANDS

#endif // _DEVICE_COMMANDS_H_