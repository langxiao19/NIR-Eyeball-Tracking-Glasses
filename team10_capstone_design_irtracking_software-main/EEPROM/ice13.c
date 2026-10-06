/**
 * @file ice14.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-08-25
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "main.h"
#include "source/app_hw/spi.h"
#include "eeprom.h"

#if defined(ICE13)

#define EEPROM_TEST_SIZE 1024

char APP_DESCRIPTION[] = "ECE353: ICE 13 - SPI EEPROM";

cyhal_trng_t trng_obj;
uint8_t Write_Values[EEPROM_TEST_SIZE];

/*****************************************************************************/
/* Peripheral Initialization                                                  */
/*****************************************************************************/

/**
 * @brief
 * This function will initialize all of the hardware resources for
 * the ICE
 */
void peripheral_init(void)
{
    task_console_init();

    /* Initialize the SPI interface */
    spi_init(EEPROM_SPI_MODULE);

    /* Initialize the EEPROM CS line*/
    eeprom_cs_init();

    /* Initialize the hardware random number generator*/
    cyhal_trng_init(&trng_obj);
}

/*****************************************************************************/
/* Application Code                                                          */
/*****************************************************************************/
/**
 * @brief
 * This function implements the behavioral requirements for the ICE
 */
void main_app(void)
{
    uint16_t address = 0x0000;
    uint8_t read_byte;

     /* Send the Clear Screen Escape Sequence*/
    printf("\x1b[2J\x1b[;H");

    printf("**************************************************\n\r");
    printf("* %s\n\r",APP_DESCRIPTION);
    printf("* Date: %s\n\r", __DATE__);
    printf("* Time: %s\n\r", __TIME__);
#ifdef NAME
    printf("* Name:%s\n\r", NAME);
#else
    printf("* Name:%s\n\r", "N/A");
#endif
    printf("**************************************************\n\r");


    printf("Generating Random Numbers\n\r");
    for (int i = 0; i < EEPROM_TEST_SIZE; i++)
    {
        Write_Values[i] = (uint8_t)cyhal_trng_generate(&trng_obj);
    }

    printf("Writing Values to EEPROM\n\r");
    for (int i = 0; i < EEPROM_TEST_SIZE; i++)
    {
        eeprom_write_byte(address, Write_Values[i]);
        address++;
    }
    address = 0;

    printf("Reading Values From EEPROM\n\r");
    for (int i = 0; i < EEPROM_TEST_SIZE; i++)
    {
        read_byte = eeprom_read_byte(address);
        if (read_byte != Write_Values[i])
        {
            printf("EEPROM Read %d ERROR... %d != %d\n\r", i, Write_Values[i], read_byte);
            CY_ASSERT(0);
        }
        else
        {
            printf("EEPROM Read %d Passed.\n\r", i);
            address++;
        }
    }

    printf("EEPROM Test Passed\n\r");
    while (1)
    {
    }
}
#endif
