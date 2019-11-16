/*
 * UART0_IRQ.c
 *
 * Created: 27.3.2015 12:28:18
 *  Author: maticpi
 */ 

#include "UART0_IRQ.h"

FILE UART0_str = FDEV_SETUP_STREAM(UART0_putc,NULL,_FDEV_SETUP_WRITE);

volatile char TxBuf0[TB_SIZE0];
int TBin0;
volatile int TBout0;
volatile int TBnum0;

volatile char RxBuf0[RB_SIZE0];
int RBin0;
volatile int RBout0;
volatile int RBnum0;

int UART0_SendStr(char str[])
{
	int i;
	for (i=0; str[i]; i++)
	{
		if (UART0_put(str[i]) != UART_OK) return i;
	}
	return i;
}

int UART0_SendBytes(char data[], int num)
{
	int i;
	for (i=0; i<num; i++)
	{
		if (UART0_put(data[i]) != UART_OK) return i;
	}
	return i;
}

int UART0_putc(char c, FILE *stream)	//for use with printf
{
	if (UART0_put(c) == UART_OK) return 0;		//All OK
	else return EOF;			//Error (EOF = -1)
}

UART_Status_t UART0_put(char data)
{
	if (UART0_numTxBytes() >= TB_SIZE0) return UART_EOF;	//Error - buffer full
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		TxBuf0[TBin0]=data;
		TBnum0++;
	}
	TBin0++;
	if (TBin0 >= TB_SIZE0) TBin0=0;
	UCSR0B |= (1<<UDRIE0);	//enable transmit IRQ
	return UART_OK;	//all OK
}

int UART0_numTxBytes()
{
	int tmp;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { tmp=TBnum0; }
	return tmp;
}

int UART0_numRxBytes()
{
	int tmp;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { tmp=RBnum0; }
	return tmp;
}

int UART0_DataReady()
{
	return (UART0_numRxBytes() > 0);
}

UART_Status_t UART0_GetByte(char* data)
{
	UART_Status_t result=UART_OK;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if (RBnum0 > 0)
		{
			*data = RxBuf0[RBout0];
			RBout0++;
			if (RBout0 >= RB_SIZE0) RBout0=0;
			RBnum0--;
		}
		else
		{
			result = UART_EOF;
		}
	}
	return result;
}

char UART0_PreviewRxByte(int index)
{
	char data=0;
	index = (index + RBout0) % RB_SIZE0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		data=RxBuf0[index];
	}
	return data;
}

ISR(USART0_RX_vect)
{
	if (RBnum0 < RB_SIZE0)
	{
		RxBuf0[RBin0]=UDR0;
		RBin0++;
		if (RBin0 >= RB_SIZE0) RBin0=0;
		RBnum0++;
	}
	//else report error - Receive buffer overflow
}

ISR(USART0_UDRE_vect)
{
	if (TBnum0 > 0)
	{
		UDR0=TxBuf0[TBout0];
		TBout0++;
		if (TBout0 >= TB_SIZE0) TBout0=0;
		TBnum0--;
	}
	else
	{
		UCSR0B &= ~(1<<UDRIE0);		//izklopi prekinitev za oddajo
	}
}

void UART0_Init()
{
	//1-output, 0-input
	DDRD |= (1<<1); //PD1 - TX0
	DDRD &= ~(1<<0); //PD0 - RX0
	
	//USART0 - UART0_ 115200 baud, 8bits, 1 stop bit, no parity, no flow control
	UCSR0A = 0;
	UCSR0C = (0<<UMSEL00) | (0<<UPM00) | (0<<USBS0) | (3<<UCSZ00);
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0) | (0<<UCSZ02);

	#ifndef BAUD0
		#warning "UART0_IRQ.c: BAUD0 not defined! Assuming 115200."
		#define BAUD0 115200UL
	#endif
	#undef BAUD
	#define BAUD BAUD0
	#include <util/setbaud.h>
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
	UCSR0A |= (1 << U2X0);
	#else
	UCSR0A &= ~(1 << U2X0);
	#endif
}