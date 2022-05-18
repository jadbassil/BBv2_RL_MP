#ifndef _FLASH_STORAGE_H_
#define _FLASH_STORAGE_H_

#include <bb_global.h>

/*!
 * \struct configuration defines the Blinky block configuration
 * After page erase all values are 0xffffffff, so we can check these and advance until we reach an empty configuration "line" to write to
 */
typedef struct _configuration {
	uint32_t flash_position;				///< The position in flash, it advances until all configuration flash page is full before erasing it
	uint32_t appli_auto_start;				///< 1 if application must auto start, 0 else
	uint32_t appli_auto_start_delay_sec;	///< how many seconds before auto start actually happens, must at least be 1
	uint32_t soft_id;						///< Blinky block soft ID
	uint32_t application_address;			///< Flash address of the application
	uint32_t reserved[11];					///< For extensions, makes 64 bytes configuration length, 32 possible instances before erasing flash
} __packed configuration;

uint8_t erase_pages(uint32_t pageAddr, uint8_t nbPages);
void reinit_configuration();
void load_configuration();
void save_configuration();
void set_parameter(uint8_t var_index, uint32_t value);

#endif // _FLASH_STORAGE_H_
