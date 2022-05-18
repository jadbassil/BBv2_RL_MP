#ifndef _SOFT_ID_H_
#define _SOFT_ID_H_

#include <bb_global.h>

#define SOFT_ID_PROTO_COUNT_CHILDREN_REQUEST 0
#define SOFT_ID_PROTO_COUNT_CHILDREN_RESPONSE 1
#define SOFT_ID_PROTO_SET_SOFT_ID_REQUEST 2
#define SOFT_ID_PROTO_SET_SOFT_ID_RESPONSE 3

/*!
 * \struct soft_id_packet contains a command byte (to indicate command required)
 * If command is a count children request, or a set ID response, there is no further content.
 * Else, the content will either be the children count (case count children response)
 * or the first ID value to use in subtree (case set soft id request)
 *
 * Another solution for soft ID definition is to use uint8_t x,y,z coordinates concatenated into a uint32_t
 */
typedef struct _soft_id_packet {
	uint8_t command;	///< Command type
	uint32_t payload;	///< Payload if required by command type
} __packed soft_id_packet;

struct _L3_packet;

uint8_t soft_id_process_packet(struct _L3_packet *p);

#endif // _SOFT_ID_H_
