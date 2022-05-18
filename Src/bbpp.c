/*
 * bbpp.c
 *
 *  Created on: 24 avr. 2020
 *      Author: flassabe
 */

#include <bbpp.h>

#ifdef BBPP

#include <stm32f0xx_hal_flash_ex.h>
#include <layer3_generic.h>
#include <layer2.h>
#include <hwLED.h>
#include <endianness.h>
#include <string.h>
#include <flash_storage.h>

#define MAX_PROGRAM_SIZE (96*1024)
#define MIN_CHUNK_SIZE 128

/*!
 * Blinky block Batch Programming Protocol
 *
 * This protocol relies on the following key principles:
 * - The programmer source (the user computer for the root node, the parent BB for any other BB)
 * sends an init_program message to its children. The init_program message contains the program
 * base address, its size, the size of the program chunks (the atomic fragment size inside one
 * packet) and the chunk count. Upon reception, the child checks whether:
 *   - it is not currently inside a programming process
 *   - the program won't overwrite its own memory
 * Only if both conditions are satisfied will the block accept to start programming. Before proceeding,
 * it initializes its pages completed map (all bits to zero). Then, the block starts asking its parent
 * for the program content. It starts by the first page, and requests all chunks one by one to the parent.
 * When a page is complete, it is written on the flash, then the block proceeds until all the program is
 * written in the flash.
 * In parallel, the block will propagate the programming parameters to its children, who, in turn,
 * proceed the same way.
 * If nothing is happening during 10 seconds subsequent to a chunk request, the programming is cancelled
 * (when writing to the bootloader, you have to flash the BB on its programming base to recover from such crash).
 * Conditions: chunks must be at least 128 bytes, program size must be at most 96 kiB (96*1024 bytes)
 */

uint8_t in_propagation = 0; // In a propagation process
uint8_t current_client = 0; // UART to which blinky is propagated
uint8_t in_programming = 0; // If a programming has started, 1, 0 else
uint8_t chunks_completed[MAX_PROGRAM_SIZE / MIN_CHUNK_SIZE / 8]; // Chunks already received in current writing page

uint32_t program_base_address; // Where the program must be stored
uint32_t program_size; // Full program size (bytes)
uint16_t chunks_size;
uint16_t chunks_count = 0; // Number of chunks for the whole program
uint16_t chunks_flashed; // Number of chunks already written to flash (and sent to neighbor)

#define IS_CHUNK_COMPLETE(i) ((chunks_completed[i/8] & (1 << (i%8))) != 0)
#define SET_CHUNK_COMPLETE(i) (chunks_completed[i/8] |= (1 << (i%8)))

extern int main(void);				// Used to check if the current program won't erase itself (would crash)
extern int8_t my_parent_uart;		// Used to query chunks
extern uint8_t my_children_uart;	// Used for propagation

L3_functions bbpp_functions = {
		.process_packet = &bbpp_process_packet,
};

BBPP_program_start copy_of_start_packet;

void request_next_chunk();

void start_propagation(uint8_t restart) {
	if (restart == 1)
		current_client = 0;
	else ++current_client;
	in_propagation = 1;
	for (; current_client<NB_SERIAL_PORT; ++current_client)
		if ((my_children_uart & (1<<current_client)) != 0)
			break;
	if (current_client < NB_SERIAL_PORT) {
		L3_packet *p = get_free_L3_packet(L3_BBPP);
		if (p) {
			p->io_port = current_client;
			p->functions = NULL;
			memcpy(p->packet_content, &copy_of_start_packet, sizeof(BBPP_program_start));
			chunks_flashed = 0;
			send_layer3_packet(p, L3_BBPP, sizeof(BBPP_program_start), 1);
		}
	}
}

uint8_t bbpp_process_packet(L3_packet *p) {
	BBPP_packet *bbpp_p = (BBPP_packet *) &p->packet_content;
	if (bbpp_p->header[0] == BBPP_PACKET_TYPE_CHUNK) {
		if (p->io_port == my_parent_uart)
			bbpp_manage_program_part((BBPP_program_part *) bbpp_p);
	} else if (bbpp_p->header[0] == BBPP_PACKET_TYPE_CHUNK_REQUEST) {
		bbpp_manage_chunk_request((BBPP_chunk_request *) bbpp_p, p->io_port);
	} else if (bbpp_p->header[0] == BBPP_PACKET_TYPE_PROGRAM_START) {
		if (p->io_port == my_parent_uart)
			bbpp_manage_program_start((BBPP_program_start *) bbpp_p);
	}
	return 0;
}

void bbpp_manage_program_start(BBPP_program_start *p) {
	memcpy(&copy_of_start_packet, p, sizeof(BBPP_program_start));
	program_base_address = switch_endian_l(p->base_address);
	program_size = switch_endian_l(p->program_size);
	chunks_size = p->chunk_size + 1;
	chunks_count = switch_endian_s(p->chunks_count);
	chunks_flashed = 0;
	if ((program_base_address <= (uint32_t) &main) && ((uint32_t)&main <= program_base_address+program_size)) { // Would erase myself
		setHWLED(50, 0, 0);
		return;
	}
	if (in_programming == 1) return; // If already programming, cannot flash another program at the same time
	// Initialize programming state
	for (uint8_t i=0; i<(MAX_PROGRAM_SIZE/MIN_CHUNK_SIZE/8); ++i)
		chunks_completed[i] = 0;
	uint8_t pages_count = program_size / FLASH_PAGE_SIZE + (program_size%FLASH_PAGE_SIZE == 0 ? 0 : 1);
	if (erase_pages(program_base_address, pages_count) != 0) return;
	in_programming = 1;
	in_propagation = 0;
	request_next_chunk();
}

void bbpp_manage_chunk_request(BBPP_chunk_request *p, uint8_t uart_idx) {
	if (in_programming == 0) { // We don't want to overload the blinky
		L3_packet *packet = get_free_L3_packet(L3_BBPP);
		if (packet) {
			packet->io_port = uart_idx;
			packet->functions = NULL;
			uint16_t chunk_idx = switch_endian_s(p->chunk_index);
			if (chunk_idx < chunks_count) { // Cannot send a non-existing chunk
				uint32_t chunk_address = program_base_address+chunk_idx*chunks_size; // Chunk address in flash
				uint16_t count_bytes = (chunk_idx==chunks_count-1) ? program_size%chunks_size : chunks_size; // bytes to copy
				BBPP_program_part *bbpp_part = (BBPP_program_part *) packet->packet_content;
				bbpp_part->pkt_type = BBPP_PACKET_TYPE_CHUNK;
				bbpp_part->chunk_index = p->chunk_index;
				bbpp_part->chunk_size = (count_bytes-1) & 0xff;
				memcpy(bbpp_part->program_bytes, (void *) chunk_address, count_bytes);
				send_layer3_packet(packet, L3_BBPP, count_bytes+4, 1);
			} else
				release_L3_packet(packet);
			if (chunk_idx == chunks_count-1)
				start_propagation(0);
		}
	}
}

void bbpp_manage_program_part(BBPP_program_part *p) {
	uint16_t chk_idx = switch_endian_s(p->chunk_index);
	if (IS_CHUNK_COMPLETE(chk_idx)) return; // Already written
	write_bytes_to_flash(program_base_address+chk_idx*chunks_size, p->program_bytes, p->chunk_size+1);
	SET_CHUNK_COMPLETE(chk_idx);
	++chunks_flashed;
	if (chunks_flashed < chunks_count) {
		request_next_chunk();
	} else {
#ifndef IS_APPLICATION
		set_parameter(4, program_base_address);
		save_configuration();
#endif
		in_programming = 0;
		setHWLED(0, 50, 0);
		start_propagation(1);
	}
}

uint8_t write_bytes_to_flash(uint32_t targetAddress, uint8_t* data, uint16_t length) {
	uint8_t retour=0;
	uint16_t i=0;
	uint16_t dataEc;
	if (length%2 == 1) { // si length est impair
		length=length-1;
	}
	while ((retour==0) && (i<length)) {
		dataEc=data[i];  // la mémoire est inversée...
		dataEc+=data[i+1]<<8;
		HAL_FLASH_Unlock();
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,targetAddress, dataEc)!=HAL_OK) { // FLASH_TYPEPROGRAM_HALFWORD=16 bits
			retour=1; // erreur
		}
		HAL_FLASH_Lock();
		targetAddress=targetAddress+2;
		i=i+2;
	}
	return retour;
}

// Private functions
void request_next_chunk() {
	uint16_t i;
	for (i=0; i<chunks_count; ++i)
		if (!IS_CHUNK_COMPLETE(i))
			break;
	L3_packet *p = get_free_L3_packet(L3_BBPP);
	if (p && my_parent_uart >= 0 && my_parent_uart < NB_SERIAL_PORT) {
		BBPP_chunk_request *bbp = (BBPP_chunk_request *) p->packet_content;
		p->io_port = my_parent_uart;
		bbp->pkt_type = BBPP_PACKET_TYPE_CHUNK_REQUEST;
		bbp->chunk_index = switch_endian_s(i);
		send_layer3_packet((L3_packet *) p, L3_BBPP, sizeof(BBPP_chunk_request), 1);
	}
}

uint8_t flash_progress() {
	if (chunks_count != 0) {
		return ((chunks_flashed+1)*100)/chunks_count;
	}
	return 0;
}

#endif // BBPP
