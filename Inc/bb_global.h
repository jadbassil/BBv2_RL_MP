#ifndef _BB_GLOBAL_H_
#define _BB_GLOBAL_H_

#include <stm32f0xx_hal.h>

//
//  /!\ You MUST modify the linker script file to have the memory mapped to 0x8010000
//
#define IS_APPLICATION

// Comment/Uncomment following defines to disable/enable features
//#define BBPP // Enables Blinky block Batch Programming Protocol
#define STP // Enables Spanning Tree Protocol
#define DEVICE_COMMANDS // Enables managing the device through UART
#define HELLO // Enables connectivity tests
//#define SOFT_ID // Setting soft ID from root
#define L3_STANDARD // Enables standard L3 packets (used by the user)

#ifdef BBPP // If BBPP is enabled, STP is required
	#ifndef STP // So if it is not enabled
		#define STP // Then we enable it to allow BBPP to forward to children
	#endif
#endif

#ifdef SOFT_ID // If SOFT_ID is enabled, STP is required
	#ifndef STP // So if it is not enabled
		#define STP // Then we enable it to allow BBPP to forward to children
	#endif
#endif

#define _packed __attribute__((__packed__))

#define NB_TX_RETRIES 3
#define FRAME_ACK_TIMEOUT_MS 100

#define NB_SERIAL_PORT 6

#define NB_BUFFER_MAX 32

#if NB_BUFFER_MAX < 9
	#define BUFFER_MASK_TYPE uint8_t
	#define BUFFER_MASK_ONES 0xff
#elif NB_BUFFER_MAX < 17
	#define BUFFER_MASK_TYPE uint16_t
	#define BUFFER_MASK_ONES 0xffff
#elif NB_BUFFER_MAX < 33
	#define BUFFER_MASK_TYPE uint32_t
	#define BUFFER_MASK_ONES 0xffffffff
#elif
	#error Max buffer size is 32
#endif

#define IS_BUFFER_FREE(buffer_mask, i) ((i<NB_BUFFER_MAX) && (((buffer_mask >> i) & 0x1) == 0x0))
#define MARK_BUFFER_USED(buffer_mask, i) do {if (i<NB_BUFFER_MAX) buffer_mask |= (1<<i);} while(0)
#define UNMARK_BUFFER_USED(buffer_mask, i) do {if (i<NB_BUFFER_MAX) buffer_mask &= BUFFER_MASK_ONES^(1<<i);} while(0)

#define CIRCULAR_LENGTH 300
typedef struct {
	uint8_t content[CIRCULAR_LENGTH];
	uint8_t pRD, pWR;
} circular_buffer;

#define BUFFER_EMPTY(circ) ((circ)->pRD == (circ)->pWR)
#define BUFFER_FULL(circ) ((((circ)->pWR+1) % CIRCULAR_LENGTH) == (circ)->pRD)
#define BUFFER_REINIT(circ) ((circ)->pRD = (circ)->pWR)
#define BUFFER_INIT(circ) ((circ)->pRD = (circ)->pWR = 0)

#endif //_BB_GLOBAL_H_
