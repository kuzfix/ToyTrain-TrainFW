/*
 * systime.h
 *
 * Created: 3.6.2015 11:52:47
 *  Author: maticpi
 *
 * Uses TIMER0 as a system counter 
 * - uint32_t systick = number of millisconds since the initialization (never read/write directly - possible race condition)
 * - uint8_t TIME_STATUS = various time related flags (never read/write directly - possible race condition)
 *
 *
 * NOTE: Requires interrupts to be enabled! 
 *       They are not enabled by default. 
 *       To enable them call sei(); after Systime_Init();
 */ 

#ifndef SYSTIME_H_
#define SYSTIME_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "config.h"

#ifndef F_CPU
#warning "systime.h: F_CPU not defined - assuming 16 MHz"
#define F_CPU	16000000UL
#endif

#if F_CPU/1000UL <= 256
	#define SYS_PRESCALLER	(1<<CS00)
	#define SYS_OCR			(F_CPU/1000UL)
#elif F_CPU/8000UL <= 256
	#define SYS_PRESCALLER	(2<<CS00)
	#define SYS_OCR			(F_CPU/8000UL)
#elif F_CPU/64000UL <= 256
	#define SYS_PRESCALLER	(3<<CS00)
	#define SYS_OCR			(F_CPU/64000UL)
#elif F_CPU/256000UL <= 256
	#define SYS_PRESCALLER	(4<<CS00)
	#define SYS_OCR			(F_CPU/256000UL)
#elif F_CPU/1024000UL <= 256
	#define SYS_PRESCALLER	(5<<CS00)
	#define SYS_OCR			(F_CPU/1024000UL)
#else
	#warning "systime.h: Clock too high! Can't find a valid prescaller settings"
	#define SYS_PRESCALLER	(5<<CS00)
	#define SYS_OCR			256
#endif


extern volatile uint8_t TIME_STATUS;
#define F1ms	0

extern volatile uint32_t systick;

#define millis	GetSysTick
#define micros	GetSysTick_us

void Systime_Init(void);		//initializes the system timer (TIMER0)
char isF1msSet(void);			//checks if 1ms flag is set
void ClrF1ms(void);				//clears the 1ms flag (set only by the ISR)
uint32_t GetSysTick(void);		//returns number of ticks/milliseconds since the initialization
uint32_t GetSysTick_us(void);	//returns number of microseconds since the initialization
char HasOneMillisecondPassed();	//returns true if 1ms flag was set and clears it
// usage:
//	if (HasOneMillisecondPassed())
//	{
//		//DO something
//	}
char Has_X_MillisecondsPassed(uint32_t x, uint32_t* start_time);	//returns TRUE if x milliseconds has passed
// usage:
//	uint32_t	mark1=0;
//	uint32_t	mark2=0;
//	systime_init();
//	sei();
//	while (1)
//	{
//		if (Has_X_MillisecondsPassed(150,&mark1))
//		{
//			//DO something
//		}
//		if (Has_X_MillisecondsPassed(1000,&mark2))
//		{
//			//DO something else
//		}
//	}
char Has_X_MicrosecondsPassed(uint32_t x, uint32_t* start_time);	//returns TRUE if x microseconds has passed
//usage: same as Has_X_MillisecondsPassed
//NOTE: resolution is about 14us with 18.432MHz crystal. 
//The system is too slow to use this function with x less than 100

#endif /* SYSTIME_H_ */