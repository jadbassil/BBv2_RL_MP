/*
 * endianness.c
 *
 *  Created on: 17 juin 2020
 *      Author: flassabe
 */

#include <endianness.h>

uint64_t switch_endian_ll(uint64_t value) {
	uint64_t value_switched;
	uint8_t *ptr_be = (uint8_t *) &value;
	uint8_t *ptr_le = (uint8_t *) &value_switched;
	for (uint8_t i=0; i<8; ++i)
		ptr_le[i] = ptr_be[7-i];
	return value_switched;
}

uint32_t switch_endian_l(uint32_t value) {
	uint32_t value_switched;
	uint8_t *ptr_be = (uint8_t *) &value;
	uint8_t *ptr_le = (uint8_t *) &value_switched;
	ptr_le[3] = ptr_be[0];
	ptr_le[2] = ptr_be[1];
	ptr_le[1] = ptr_be[2];
	ptr_le[0] = ptr_be[3];
	return value_switched;
}

uint16_t switch_endian_s(uint16_t value) {
	uint16_t value_switched;
	uint8_t *ptr_source = (uint8_t *) &value;
	uint8_t *ptr_dest = (uint8_t *) &value_switched;
	ptr_dest[0] = ptr_source[1];
	ptr_dest[1] = ptr_source[0];
	return value_switched;
}

