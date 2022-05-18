#ifndef _ENDIANNESS_H_
#define _ENDIANNESS_H_

#include <bb_global.h>

uint64_t switch_endian_ll(uint64_t value);
uint32_t switch_endian_l(uint32_t value);
uint16_t switch_endian_s(uint16_t value);

#endif // _ENDIANNESS_H_
