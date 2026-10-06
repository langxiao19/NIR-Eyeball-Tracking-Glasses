#ifndef TASK_EEPROM_H__
#define TASK_EEPROM_H__

#include "main.h"
#include "spi.h"
#include "ece453_pins.h"

#define EEPROM_SPI_MODULE   MODULE_SITE_0
#define EEPROM_CS_PIN       MOD_0_PIN_SPI_CS_N   /* P10_3, active-low */
#define EEPROM_HOLD_PIN     MOD_0_PIN_IO_0        /* P5_4, drive HIGH to deassert HOLD# */
#define EEPROM_WP_PIN       MOD_0_PIN_IO_1        /* P5_5, drive HIGH to deassert WP# */
#define EEPROM_USE_HW_CS    0u
/* Test switch: set 0 to test "CS logic is reversed" hypothesis */
#define EEPROM_CS_ACTIVE_LOW 1u
#define EEPROM_PAGE_SIZE    128u
/* Address phase length: 1/2/3 depending on EEPROM density/protocol */
#define EEPROM_ADDR_BYTES   2u

cy_rslt_t eeprom_init(void);
cy_rslt_t eeprom_read(uint16_t addr, uint8_t *buf, uint16_t len);
cy_rslt_t eeprom_write(uint16_t addr, const uint8_t *data, uint16_t len);
void task_eeprom_init(void);

#endif /* TASK_EEPROM_H__ */
