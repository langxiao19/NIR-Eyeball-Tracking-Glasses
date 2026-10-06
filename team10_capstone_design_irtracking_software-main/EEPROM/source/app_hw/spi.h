/*
 *  Created on: Jan 18, 2022
 *      Author: Joe Krachey
 */

#ifndef SPI_H__
#define SPI_H__

#include "main.h"
#include "ece453_pins.h"

#define SPI_FREQ			10000
/* M95xx EEPROM supports mode 0/3; keep mode 0 by default and make it easy to switch */
#define SPI_MODE_EEPROM    CYHAL_SPI_MODE_00_MSB

/* Public Global Variables */
extern cyhal_spi_t mSPI;
extern SemaphoreHandle_t Semaphore_SPI;

/* Public API */
cy_rslt_t spi_init(module_site_t module_site);

#endif 
