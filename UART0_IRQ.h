/*
 * UART0_IRQ.h
 *
 * Created: 27.3.2015 12:28:35
 *  Author: maticpi
 */ 


#ifndef UART0_IRQ_H_
#define UART0_IRQ_H_

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "config.h"

/*//Set your own names of the functions for each UART
#define _Init() UART0_Init()
#define _SendStr(x) UART0_SendStr(x)
#define _SendBytes(x,y) UART0_SendBytes(x,y)
#define _putc(x,y) UART0_putc(x,y)
#define _DataReady() UART0_DataReady()
#define _GetByte() UART0_GetByte()
#define _put(x) UART0_put(x)
//#define _numTxBytes() UART0_numTxBytes()	//not much point really
#define _numRxBytes() UART0_numRxBytes()
#define _PreviewRxByte(x) UART0_PreviewRxByte(x)
#define _str UART0_str
*/
typedef enum {
	UART_EOF	= -1,
	UART_OK		= 0
} UART_Status_t;

void UART0_Init();
int UART0_SendStr(char str[]);
int UART0_SendBytes(char data[], int num);
int UART0_putc(char c, FILE *stream);
int UART0_DataReady();
UART_Status_t UART0_GetByte(char* data);
UART_Status_t UART0_put(char data);
int UART0_numTxBytes();
int UART0_numRxBytes();
char UART0_PreviewRxByte(int index);

extern FILE UART0_str;

#define TB_SIZE0	500
extern volatile char TxBuf0[TB_SIZE0];
extern int TBin0;
extern volatile int TBout0;
extern volatile int TBnum0;

#define RB_SIZE0	100
extern volatile char RxBuf0[RB_SIZE0];
extern int RBin0;
extern volatile int RBout0;
extern volatile int RBnum0;

#endif /* UART0_IRQ_H_ */