/*
 * LCD.h
 *
 * Created: 23.4.2013 22:14:06
 *  Author: maticpi
 */ 

#ifndef KBD_H_
#define KBD_H_

#include <avr/io.h>			//header file with register definitions
/* BTN1 = PC2 */

#define BTN1	1

#define BTN1_DDR	DDRC

#define BTN1_PORT	PORTC

#define BTN1_PIN	PINC

#define BTN1_BIT	2

void KBD_Init();
void KBD_Read();
char KBD_GetKey();
char KBD_GetReleasedKey();
void KBD_flush();
char KBD_isKeyStatePressed(char key);
char KBD_isKeyStateReleased(char key);

extern char lastkey;
extern char lastreleased;

#endif /* KBD_H_ */