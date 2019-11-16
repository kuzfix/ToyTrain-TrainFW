/*
 * ADC.c
 *
 * Created: 12-Mar-18 09:14:08
 *  Author: maticpi
 */ 
#include "ADC.h"

int ADC_ReadChannel(char channel)
{
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F );
	_delay_us(10);
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC)) {}
	return ADC;
}

void ADC_SelectChannel(char channel)
{
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F );
}

int ADC_Read()
{
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC)) {}
	return ADC;
}

void ADC_Init()
{
	DIDR0=0x0F;
	ADMUX=(1<<REFS0);
	ADCSRA=(1<<ADEN) | (7<<ADPS0);
}