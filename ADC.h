/*
 * ADC.h
 *
 * Created: 12-Mar-18 09:14:25
 *  Author: maticpi
 */ 

#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include "config.h"
#include <util/delay.h>

int ADC_ReadChannel(char channel);		//change channel, wait 10us and read
void ADC_SelectChannel(char channel);	//change channel (DOES NOT WAIT FOR THE NEW VALUE TO STABALIZE - you have to wait a few us before calling ADC_Read() )
int ADC_Read();							//read the previously selected channel	
void ADC_Init();						//Init ADC, select VCC as a reference source


#endif /* ADC_H_ */