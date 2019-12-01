/*
 * kbd.c
 *
 * Created: 5.6.2014 16:45:06
 *  Author: maticpi
 */ 

#include "kbd.h"

char lastkey;
char lastreleased;

void KBD_Init()
{
	BTN1_PORT |= (1<<BTN1_BIT);

	BTN1_DDR &=~ (1<<BTN1_BIT);
}

char KBD_isKeyStatePressed(char key)
{
	char result=0;
	switch (key)
	{
		case BTN1: result = (BTN1_PIN & (1<<BTN1_BIT)) != (1<<BTN1_BIT); break;
	}
	return result;
}

char KBD_isKeyStateReleased(char key)
{
	char result=0;
	switch (key)
	{
		case BTN1: result = (BTN1_PIN & (1<<BTN1_BIT)) == (1<<BTN1_BIT); break;
	}
	return result;
}

void KBD_Read()
{
	static char oldState=0xff;						//holds the old value of the keyboard IO port
	char newState;
	char pressed;
	char released;

	//get the new value of the IO port
	newState = 0;
	newState |= KBD_isKeyStatePressed(BTN1)<<0;
	
	pressed = (newState ^ oldState) & oldState;	//if the port state has changed, and the old value was 1, the key was pressed
	released = (newState ^ oldState) & newState;	//if the port state has changed, and the new value is 1, the key was released
	
	if (pressed & (1<<0)) lastkey=BTN1;			//if the corresponding bit in variable "pressed" is one, then that key was pressed
	
	if (released & (1<<0)) lastreleased=BTN1;			//if the corresponding bit in variable "released" is one, then that key was pressed
	
	oldState=newState;								//update the 
}

char KBD_GetKey()
{
	char tmp=lastkey;
	lastkey=0;
	return tmp;
}

char KBD_GetReleasedKey()
{
	char tmp=lastreleased;
	lastreleased=0;
	return tmp;
}

void KBD_flush()
{
	lastkey=0;
	lastreleased=0;
}
