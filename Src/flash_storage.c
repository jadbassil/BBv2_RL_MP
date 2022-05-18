/*
 * flash_storage.c
 *
 *  Created on: 23 juin 2020
 *      Author: flassabe
 */

#include <flash_storage.h>
#include <string.h>

configuration flash_config_mirror[FLASH_PAGE_SIZE / sizeof(configuration)]; ///< Used to copy in RAM the actual state of the configuration

configuration *my_configuration = NULL; ///< the actual configuration, searched in the flash
uint8_t config_changed = 0; ///< 1 if config must be written, 0 else

uint8_t erase_pages(uint32_t pageAddr, uint8_t nbPages) {
	uint8_t retour=0;
	if (HAL_FLASH_Unlock()!=HAL_OK) { // unlock flash
		retour=1;
	} else {
		FLASH_EraseInitTypeDef EraseInit;
		uint32_t uiPageError;
		EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
		EraseInit.PageAddress = pageAddr;
		EraseInit.NbPages = nbPages;
		if (HAL_FLASHEx_Erase(&EraseInit, &uiPageError)!=HAL_OK) {
			retour=1;
		}
	}
	if (HAL_FLASH_Lock()!=HAL_OK) {
		retour=1;
	}
	return retour;
}

void reinit_configuration() {
	erase_pages(0x8007800, 1);
	load_configuration();
}

void load_configuration() {
	memcpy(flash_config_mirror, (void *) 0x8007800, FLASH_PAGE_SIZE);
	uint8_t i = 0;
	while ((i+1)<(FLASH_PAGE_SIZE / sizeof(configuration)) && flash_config_mirror[i+1].flash_position == (i+1)) ++i;
	if (i == 0) {
		my_configuration = &flash_config_mirror[0];
		if (my_configuration->flash_position != 0) { // Not a valid config: initialize with default values
			my_configuration->flash_position = 0;
			my_configuration->appli_auto_start = 0;
			my_configuration->appli_auto_start_delay_sec = 1;
			my_configuration->soft_id = 0;
			my_configuration->application_address = 0x8008000;
			config_changed = 1;
		}
	}
	if (!config_changed) {
		// Move configuration pointer to next entry
		my_configuration = &flash_config_mirror[(i+1)%(FLASH_PAGE_SIZE / sizeof(configuration))];
		memcpy(my_configuration, &flash_config_mirror[i], sizeof(configuration));
		my_configuration->flash_position = (i+1)%(FLASH_PAGE_SIZE / sizeof(configuration));
		// Check values
		if (my_configuration->appli_auto_start != 0 && my_configuration->appli_auto_start != 1) {
			my_configuration->appli_auto_start = 0;
			config_changed = 1;
		}
		if (my_configuration->appli_auto_start_delay_sec == 0 || my_configuration->appli_auto_start_delay_sec > 10) {
			my_configuration->appli_auto_start_delay_sec = 10;
			config_changed = 1;
		}
		if (my_configuration->application_address < 0x8008000 || my_configuration->application_address >= 0x8020000) {
			my_configuration->application_address = 0x8008000;
			config_changed = 1;
		}
	}
	if (config_changed)
		save_configuration();
}

void save_configuration() {
	if (config_changed) { // Only save is there is a change
		if (my_configuration->flash_position == 0) { // We must erase the page
			erase_pages(0x8007800, 1);
		}
		uint32_t *ptr = (uint32_t *) my_configuration;
		for (uint8_t i=0; i<16; ++i) {
			HAL_FLASH_Unlock();
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x8007800+(my_configuration->flash_position*sizeof(configuration))+(4*i), ptr[i]);
			HAL_FLASH_Lock();
		}
		// Move configuration pointer to the next entry, and copy last configuration values
		uint8_t new_position = (my_configuration->flash_position + 1) % (FLASH_PAGE_SIZE/sizeof(configuration));
		memcpy((void *) &flash_config_mirror[new_position], (void *) my_configuration, sizeof(configuration));
		my_configuration = &flash_config_mirror[new_position];
		my_configuration->flash_position = new_position;
		config_changed = 0;
	}
}

void set_parameter(uint8_t var_index, uint32_t value) {
	if (var_index > 15) return;
	uint32_t *ptr = (uint32_t *) my_configuration;
	if (*(ptr+var_index) != value) {
		*(ptr+var_index) = value;
		config_changed = 1;
	}
}
