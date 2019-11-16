/* 
 * Please find the license in the GIT repo.
 * 
 * Description:
 * 
 * The library contains Low-level functions for the 
 * target processor. The library is designed to be used to
 * interface the NRF24L01 module with the processor.
 * 
 * Git repo:
 * 
 * https://github.com/pradeepa-s/pdlib_nrf24l01.git
 * 
 * Contributors:
 * 
 * Pradeepa Senanayake : pradeepa.kck@gmail.com
 * 
 * Change log:
 * 
 * 2014-07-09 : Support for LM4F120H5QR processor
 * 				Added configurations required for Freescale SPI
 * 				Added GPIO configurations for all SSI modules
 * 				Added function to get one byte(blocking and none blocking)
 * 				Added function to send data(blocking)
 * 
 */

#include <stdio.h>
#include "pdlib_LL.h"

#include <avr/io.h>
/* PIN AND REGISTER DEFINITIONS */

#define PDLIB_SPCR	SPCR0
#define PDLIB_SPSR	SPSR0
#define PDLIB_SPDR	SPDR0

#define PDLIB_DDR_MISO	DDRB
#define PDLIB_PORT_MISO	PORTB
#define PDLIB_PIN_MISO	PINB
#define PDLIB_BIT_MISO	4
#define PDLIB_DDR_MOSI	DDRB
#define PDLIB_PORT_MOSI	PORTB
#define PDLIB_BIT_MOSI	3
#define PDLIB_DDR_SCK	DDRB
#define PDLIB_PORT_SCK	PORTB
#define PDLIB_BIT_SCK	5
#define PDLIB_DDR_CS	DDRB
#define PDLIB_PORT_CS	PORTB
#define PDLIB_BIT_CS	0
#define PDLIB_DDR_CE	DDRD
#define PDLIB_PORT_CE	PORTD
#define PDLIB_BIT_CE	7

#define PDLIB_DDR_IRQ	  DDRD
#define PDLIB_PORT_IRQ	PORTD
#define PDLIB_BIT_IRQ	  2


#define PDLIB_CE_LOW()	PDLIB_PORT_CE &= ~(1<<PDLIB_BIT_CE)
#define PDLIB_CE_HIGH()	PDLIB_PORT_CE |=  (1<<PDLIB_BIT_CE)

#define PDLIB_CS_LOW()	PDLIB_PORT_CS &= ~(1<<PDLIB_BIT_CS)
#define PDLIB_CS_HIGH()	PDLIB_PORT_CS |=  (1<<PDLIB_BIT_CS)

// ----------------  Hardware Pin Control ------------------ //


/* PS:
 *
 * Function		: 	NRF24L01_LL_CELow
 *
 * Arguments	: 	None
 *
 * Return		: 	None
 *
 * Description	: 	This function drives the CE pin low
 *
 */

void
NRF24L01_LL_CELow()
{
	PDLIB_CE_LOW();
}


/* PS:
 *
 * Function		: 	NRF24L01_LL_CEHigh
 *
 * Arguments	: 	None
 *
 * Return		: 	None
 *
 * Description	: 	This function drives the CE pin high
 *
 */

void
NRF24L01_LL_CEHigh()
{
	PDLIB_CE_HIGH();
}


/* PS:
 *
 * Function		: 	NRF24L01_LL_CSNLow
 *
 * Arguments	: 	None
 *
 * Return		: 	None
 *
 * Description	: 	This function drives the CSN pin low
 *
 */

void
NRF24L01_LL_CSNLow()
{
	PDLIB_CS_LOW();
}


/* PS:
 *
 * Function		: 	NRF24L01_LL_CSNHigh
 *
 * Arguments	: 	None
 *
 * Return		: 	None
 *
 * Description	: 	This function drives the CSN pin high
 *
 */

void
NRF24L01_LL_CSNHigh()
{
	PDLIB_CS_HIGH();
}



/* PS:
 * 
 * Function		: 	NRF24L01_LL_ConfigureSPIInterface
 * 
 * Arguments	: 	ucSSI - SSI module index (0,1,2,3,4,5) [5 is for SSI1 with GPIO PORTD]
 * 
 * Return		: 	None
 * 
 * Description	: 	The function will setup the SSI module to make it same as Freescale SPI module.
 * 					GPIO configurations required and Clock configurations also done in the function.
 * 
 */

void 
NRF24L01_LL_ConfigureSPIInterface()
{
	PDLIB_DDR_MOSI |= (1<<PDLIB_BIT_MOSI);	//MOSI set as output
	PDLIB_DDR_MISO &= ~(1<<PDLIB_BIT_MISO);	//MISO set as input
	PDLIB_PORT_MISO &= ~(1<<PDLIB_BIT_MISO);	//pull-up on MISO off
	PDLIB_DDR_SCK |= (1<<PDLIB_BIT_SCK);	//SCK set as output
	PDLIB_DDR_CS |= (1<<PDLIB_BIT_CS);	//CS set as output
	
	PDLIB_SPCR = (0<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<< CPOL) | (0<<CPHA) | (0<<SPR0);
	PDLIB_SPSR = (1<<SPI2X);
}

/* PS:
 *
 * Function		: 	NRF24L01_LL_InterruptInit
 *
 * Arguments	:	None
 *
 * Return		: 	None
 *
 * Description	:	Registers the IRQ pin as an interrupt
 *
 */

#ifdef NRF24L01_CONF_INTERRUPT_PIN

void 
NRF24L01_LL_InterruptInit(){
//TODO: ...
}

#endif

/* PS:
 * 
 * Function		: 	NRF24L01_LL_TransferByte
 *
 * Arguments	: 	ucData	:	Data byte to transfer
 *
 * Return		: 	Function will return whatever data received from the module during the transfer.
 *
 * Description	: 	The function will submit a data byte the SPI module
 * 					for transmission and will wait until the total transmission
 * 					is over. Then it will read the TX buffer and read one byte out from the buffer.
 * 					This will clear the TX buffer.
 *
 */

unsigned char 
NRF24L01_LL_TransferByte(unsigned char ucData)
{
	uint8_t result=0;

	PDLIB_SPDR = ucData;
	while(!(PDLIB_SPSR & (1<<SPIF)));
	result = PDLIB_SPDR;
	return result;
}
