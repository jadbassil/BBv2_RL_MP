/*
 * user_code.c
 *
 *  Created on: 5 oct. 2020
 *      Author: jbassil
 */
#include <BB.h>
#include <hwLED.h>
#include <bb_global.h>
#include <serial.h>
#include <layer3_generic.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define BLUE_LEADER 1
#define WHITE_LEADER 100
#define RED_LEADER 1000

typedef enum portReferences { NORTH, DOWN, EAST, WEST, SOUTH, UP, NUM_PORTS, UNDEFINED_PORT = 255 }  enum___portReferences;
typedef enum MyColors {Blue, White, Red, Undefined} enum__MyColors;


int8_t Blue_Array[20] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,1,1};
int8_t White_Array[20] = {1, 1, -1, -1, 1, 1, -1, -1,1, 1, -1, -1, 1, 1, -1, -1,1, 1, -1, -1};
int8_t Red_Array[20] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1};


int8_t WT[10][20] = {
		{ 3, 3, 1, 1, 3, 3, 1, 1, 3, 3, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1 },
		{ 3, 3, 1, 1, 3, 3, 1, 1, 3, 3, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1 },
		{ 1, 1, 3, 3, 1, 1, 3, 3, 1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1 },
		{ 1, 1, 3, 3, 1, 1, 3, 3, 1, 1, 1, 1, -1, 1, 1, -1, -1, 1, 1 },
		{ 3, 3, 1, 1, 3, 3, 1, 1, 3, 3, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1 },
		{ 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 3, 3, 1, 1, 3, 3, 1, 1 },
		{ -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 3, 3, 1, 1, 3, 3, 1, 1, 3, 3 },
		{ -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 3, 3, 1, 1, 3, 3, 1, 1, 3, 3 },
		{ 1, 1, -1, -1,	1, 1, -1, -1, 1, 1, 1, 1, 3, 3, 1, 1, 3, 3, 1, 1},
		{ 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, 1, 1, 3, 3, 1, 1, 3, 3, 1, 1 }
};

enum__MyColors MyColor;

int8_t Sign(float x) {
	if(x >= 0) return 1;
	return -1;
}

void dataByWT(int8_t* data, int8_t size, int8_t* result_vector) {
	for(uint8_t i=0; i<20; i++) {
		result_vector[i] = 0;
		for(uint8_t j=0; j< 10; j++) {
			result_vector[i] += data[j] * WT[j][i];
		}
        result_vector[i] = Sign(result_vector[i]);
	}
}

void standard_unack_handler(L3_packet *p) {
//	setColor(ORANGE);
}

void float2Bytes(unsigned char bytes_temp[4],float float_variable){
	union {
		float a;
		unsigned char bytes[4];
	} thing;
	thing.a = float_variable;
	memcpy(bytes_temp, thing.bytes, 4);
}

void uint32_t2Bytes(unsigned char bytes_tmp[4], uint32_t uint32_t_variable) {
	union {
		uint32_t a;
		unsigned char bytes[4];
	} thing;
	thing.a = uint32_t_variable;
	memcpy(bytes_tmp, thing.bytes, 4);
}


void delay(int dt) {
    uint32_t end_t = HAL_GetTick()+dt;
    while (HAL_GetTick() < end_t) {
    }
}


uint8_t cmpDataToColor(int8_t* data, int8_t size, enum__MyColors c) {
	int8_t* colorArray;
	switch(c) {
	case Blue: {
		colorArray = Blue_Array;
	} break;
	case White: {
		colorArray = White_Array;
	} break;
	case Red: {
		colorArray = Red_Array;
	}break;
	default: return 0;
	}
	for(uint8_t i = 0; i < size; i++) {
		if(data[i] != colorArray[i])
			return 0;
	}
	return 1;
}

enum__MyColors getColorFromMessage(L3_packet* packet) {
	int8_t data[10];
	for(uint8_t i=0; i<10; i++) {
//		if(packet->packet_content[i] == 0)
//			data[i] = -1;
//		else if(packet->packet_content[i] == 1) {
//			data[i] = 1;
//		}
		data[i] = (int8_t) packet->packet_content[i];
	}
	int8_t *result_array = (int8_t*) calloc(20, sizeof(int8_t));
	dataByWT(data, 10, result_array);
	if(cmpDataToColor(result_array, 20, Red) == 1) return Red;
	if(cmpDataToColor(result_array, 20, White) == 1) return White;
	if(cmpDataToColor(result_array, 20, Blue) == 1) return Blue;
	return Undefined;

}

void mySetColor(enum__MyColors c) {
	switch(c) {
		case Blue: {
			setColor(BLUE);
		} break;
		case White: {
			setColor(WHITE);
		} break;
		case Red: {
			setColor(RED);
		}break;
		default: setColor(BLACK);
		}
}

uint8_t sendRedMessage(uint8_t port) {
	uint8_t data[10] = {255, 255, 255, 255, 255, 1, 1, 1, 1, 1};
	sendMessage(port, data, 10, 1);
	return 1;
}

uint8_t sendBlueMessage(uint8_t port) {
	uint8_t data[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	sendMessage(port, data, 10, 1);
	return 1;
}

uint8_t sendWhiteMessage(uint8_t port) {
	uint8_t data[10] = { 1, 1, 255, 255, 1, 1, 255, 255, 1, 1 };
	sendMessage(port, data, 10, 1);
	return 1;
}

void msgHandler(L3_packet* packet) {
	if(MyColor != Undefined) return;
	enum__MyColors c = getColorFromMessage(packet);
	if(c == Blue && getId() >= 100) return;
	if(c == White && (getId() < 100 || getId() >= 1000)) return;
	if(c == Red && (getId() < 1000)) return;
	MyColor = c;
	mySetColor(MyColor);
	//send the message to all neighbors except sender
	for(uint8_t p = 0; p < NB_SERIAL_PORT; p++) {
		if(is_connected(p) && p != packet->io_port) {
			sendMessage(p, packet->packet_content, 10, 1);
		}
	}
}



void BBinit() {
	MyColor = Undefined;
	setHWLED(0, 0, 0);
	if(getId() == BLUE_LEADER) {
		MyColor = Blue;
		setColor(BLUE);
		delay(1000);
		for(uint8_t p = 0; p < NB_SERIAL_PORT; p++) {
			if(is_connected(p)) {
				sendBlueMessage(p);
			}
		}
	} else if(getId() == WHITE_LEADER) {
		MyColor = White;
		setColor(WHITE);
		delay(1000);
		for(uint8_t p = 0; p < NB_SERIAL_PORT; p++) {
			if(is_connected(p)) {
				sendWhiteMessage(p);
			}
		}
	} else if(getId() == RED_LEADER) {
		MyColor = Red;
		setColor(RED);
		delay(1000);
		for(uint8_t p = 0; p < NB_SERIAL_PORT; p++) {
			if(is_connected(p)) {
				sendRedMessage(p);
			}
		}
	}
}


void BBloop() {


}

uint8_t process_standard_packet(L3_packet *packet) {

	msgHandler(packet);
    return 0;
}

