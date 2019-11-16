/*
 * systime.c
 *
 * Created: 3.6.2015 11:53:00
 *  Author: maticpi
 */ 

#include "systime.h"
volatile uint8_t TIME_STATUS;
volatile uint32_t systick;

ISR(TIMER0_COMPA_vect)
{
	systick++;
	TIME_STATUS |= (1<<F1ms);	//SetF1ms()
}

void Systime_Init()
{
	TCCR0A = (2<<WGM00);		//normal port function for OC0A & OC0B, Mode = CTC
	TCCR0B = SYS_PRESCALLER;	//Set prescaller and OCR0A to trig every 1 ms
	OCR0A = SYS_OCR-1;
	TIMSK0 = (1<<OCIE0A);
}

uint32_t GetSysTick(void)
{
	uint32_t tick;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		tick = systick;
	}
	return tick;
}

uint32_t GetSysTick_us(void)
{
	uint32_t tick, tick_us, tim;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		tim = TCNT0;
		tick = systick;
	}
	tick_us=(uint32_t)((uint64_t)tick*1000 + (uint64_t)((tim*1000)/SYS_OCR));

	return tick_us;
}

char HasOneMillisecondPassed()
{
	if (isF1msSet())
	{
		ClrF1ms();
		return 1;
	}
	return 0;
}

char Has_X_MillisecondsPassed(uint32_t x, uint32_t* start_time)
{
	uint32_t now = GetSysTick();

	if (x == 0) return 0;		//x==0 is not a valid value. Abort.
	if (*start_time == 0)		//if first call, prepare the start_time variable
	{
		*start_time = now;
		return 0;				//signal it is not time to do anything
	}
	if (now > *start_time + 4*x)	//if we are very late, adjust start_time to now
	{
		*start_time = now;
		return 3;				//signal LATE and TRUE;
	}
	if (now >= *start_time + x)	//if the time has passed, prepare start_time for next cycle
	{
		*start_time += x;
		return 1;				//signal TRUE;
	}
	if (now < *start_time )		//if start_time is in the future, adjust it to now
	{
		*start_time = now;
		return 0;				//signal FALSE;
	}
	return 0;	//target time has not been reached - signal FALSE
}

char Has_X_MicrosecondsPassed(uint32_t x, uint32_t* start_time)
{
	uint32_t now = GetSysTick_us();

	if (x == 0) return 0;		//x==0 is not a valid value. Abort.
	if (*start_time == 0)		//if first call, prepare the start_time variable
	{
		*start_time = now;
		return 0;				//signal it is not time to do anything
	}
	if (now > *start_time + 4*x)	//if we are very late, adjust start_time to now
	{
		*start_time = now;
		return 3;				//signal LATE and TRUE;
	}
	if (now >= *start_time + x)	//if the time has passed, prepare start_time for next cycle
	{
		*start_time += x;
		return 1;				//signal TRUE;
	}
	if (now < *start_time )		//if start_time is in the future, adjust it to now
	{
		*start_time = now;
		return 0;				//signal FALSE;
	}
	return 0;	//target time has not been reached - signal FALSE
}

char isF1msSet(void)
{
	char result;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		result = TIME_STATUS & (1<<F1ms);
	}
	return result;
}

void ClrF1ms(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		TIME_STATUS &= ~(1<<F1ms);
	}
}