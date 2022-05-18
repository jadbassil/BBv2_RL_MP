#ifndef _BBPP_H_
#define _BBPP_H_

#include <bb_global.h>

#ifdef BBPP

#include <stm32f0xx_hal.h>

#define MAX_BBPP_CONTENT_LENGTH 256

#define BBPP_PACKET_TYPE_CHUNK 0
#define BBPP_PACKET_TYPE_CHUNK_REQUEST 1 // Request a chunk from the father
#define BBPP_PACKET_TYPE_CHUNK_REQUEST_UNAVAILABLE 2 // Response to chunk request: chunk not available
#define BBPP_PACKET_TYPE_CHUNK_REQUEST_NOEXIST 3 // Response to chunk request: chunk does not exist
#define BBPP_PACKET_TYPE_PROGRAM_START 4

typedef struct _BBPP_packet {
	uint8_t header[4];
	uint8_t program_bytes[MAX_BBPP_CONTENT_LENGTH];
} __packed BBPP_packet;

// Types for specific types of packets, must be cast from a BBPP_packet pointer
typedef struct {
	uint8_t pkt_type;
	uint32_t base_address;
	uint32_t program_size;
	uint8_t chunk_size; // Actually, it is chunk_size-1, so it ranges from 1 to 256 on 1 byte
	uint16_t chunks_count; // Number of chunks for the whole program
//	uint16_t program_crc16; // To avoid program chunks mismatch (who knows...)
} __packed BBPP_program_start;

typedef struct {
	uint8_t pkt_type;
//	uint16_t program_crc16;
	uint16_t chunk_index;
} __packed BBPP_chunk_request;

typedef struct {
	uint8_t pkt_type;
	uint16_t chunk_index;
	uint8_t chunk_size; // chunk_size-1, so it ranges from 1 to 256 on 1 byte
	uint8_t program_bytes[MAX_BBPP_CONTENT_LENGTH];
} __packed BBPP_program_part;

struct _L3_packet;

// TODO: move to specialized compilation unit
uint8_t write_bytes_to_flash(uint32_t targetAddress, uint8_t* data, uint16_t length);

uint8_t bbpp_process_packet(struct _L3_packet *p);
void bbpp_manage_program_start(BBPP_program_start *p);
void bbpp_manage_chunk_request(BBPP_chunk_request *p, uint8_t uart_idx);
void bbpp_manage_program_part(BBPP_program_part *p);
uint8_t flash_progress();

#endif // BBPP

#endif // _BBPP_H_
