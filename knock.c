/****************************************************************
 *       SECU-3  - An open source, free engine control unit
 *    Designed by Alexey A. Shabelnikov. Ukraine, Gorlovka 2007.
 *       Microprocessors systems - design & programming.
 *    contacts:
 *              http://secu-3.narod.ru
 *              ICQ: 405-791-931
 ****************************************************************/
#include <inavr.h>
#include <iom16.h>
#include "knock.h"


void knock_set_band_pass(unsigned char freq)
{
}

void knock_set_attenuation(unsigned char attenuation)
{
}

void knock_set_integration_time(unsigned char inttime)
{
}

void knock_module_initialize(void)
{
}

void spi_master_init(void)
{
 // Set MOSI and SCK output, all others do not change 
 DDRB|= (1 << DDB5)|(1 << DDB7);
 // Enable SPI, Master, set clock rate fck/16
 SPCR = (1 << SPE)|(1 << MSTR)|(1 << SPR0);
}

void spi_master_transmit(char cdata)
{
 // Start transmission
 SPDR = cdata;
 // Wait for transmission complete
 while(!(SPSR & (1 << SPIF)));
}
