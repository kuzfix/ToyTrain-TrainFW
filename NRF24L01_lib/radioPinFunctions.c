/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* Please define your platform spesific functions in this file ...
* -----------------------------------------------------------------------------
*/

#include <avr/io.h>

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

#define set_bit(reg,bit) reg |= (1<<bit)
#define clr_bit(reg,bit) reg &= ~(1<<bit)
#define check_bit(reg,bit) (reg&(1<<bit))

/* ------------------------------------------------------------------------- */
void nrf24_setupPins()
{
    set_bit(PDLIB_DDR_CE,PDLIB_BIT_CE); // CE output
    set_bit(PDLIB_DDR_CS,PDLIB_BIT_CS); // CSN output
    set_bit(PDLIB_DDR_SCK,PDLIB_BIT_SCK); // SCK output
    set_bit(PDLIB_DDR_MOSI,PDLIB_BIT_MOSI); // MOSI output
    clr_bit(PDLIB_DDR_MISO,PDLIB_BIT_MISO); // MISO input
}
/* ------------------------------------------------------------------------- */
void nrf24_ce_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(PDLIB_PORT_CE,PDLIB_BIT_CE);
    }
    else
    {
        clr_bit(PDLIB_PORT_CE,PDLIB_BIT_CE);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_csn_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(PDLIB_PORT_CS,PDLIB_BIT_CS);
    }
    else
    {
        clr_bit(PDLIB_PORT_CS,PDLIB_BIT_CS);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_sck_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(PDLIB_PORT_SCK,PDLIB_BIT_SCK);
    }
    else
    {
        clr_bit(PDLIB_PORT_SCK,PDLIB_BIT_SCK);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_mosi_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(PDLIB_PORT_MOSI,PDLIB_BIT_MOSI);
    }
    else
    {
        clr_bit(PDLIB_PORT_MOSI,PDLIB_BIT_MOSI);
    }
}
/* ------------------------------------------------------------------------- */
uint8_t nrf24_miso_digitalRead()
{
    return check_bit(PDLIB_PIN_MISO,PDLIB_BIT_MISO);
}
/* ------------------------------------------------------------------------- */
