#ifndef _STP_H_
#define _STP_H_


#include <stm32f0xx_hal.h>
#include <bb_global.h>
#include <layer3_generic.h>

#ifdef STP

/*!
 * \brief stp_command is the only byte required to be transmitted by the STP protocol
 * It is decomposed into two parts:
 *  - bits 7 and 6 define the command type:
 *    * 00 (0x00) -> (request) clear tree
 *    * 01 (0x40) -> (request) source tries to be parent of destination
 *    * 10 (0x80) -> (response) destination accepts being child of source
 *    * 11 (0xc0) -> (response) destination refuses to be child of source
 *  - other bits define the spanning tree identifier (not implemented yet)
 */
typedef uint8_t stp_command;

uint8_t process_STP_packet(L3_packet *p);

void copy_packet_to_children(L3_packet *p, L3_packet_type ptype, uint16_t size);

uint8_t is_child(uint8_t uart_index);

#endif // STP

#endif // _STP_H_
