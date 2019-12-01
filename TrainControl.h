/*
 * TrainControl.h
 *
 * Created: 01/12/2019 12:13:03
 *  Author: maticpi
 */ 


#ifndef TRAINCONTROL_H_
#define TRAINCONTROL_H_

//Includes only for debugging (I think):
#include "config.h"
#include <util/delay.h>
#include "UART0_IRQ.h"

#include "systime.h"
#include "HWmotorControl.h"

#define CMD_FORWARD           1
#define CMD_BACK              2
#define CMD_SND_TRG_START     3
#define CMD_SND_TRG_STOP      4
#define CMD_BLOCK             5
#define CMD_PENDING_BITMASK 0x80

#define SndTrgOn() {DDRD |= (1<<6); PORTD |= (1<<6); DbgOn();}     //Has to be "1"
#define SndTrgOff() {DDRD &= ~(1<<6); PORTD &= ~(1<<6); DbgOff();}  //          or "HZ" (no pull-up)
#define DbgOn() PORTD |= (1<<5)
#define DbgOff() PORTD &= ~(1<<5)

void ProcessCommandQueue();
void TrainForward(int speed);
void StopTrain();

#endif /* TRAINCONTROL_H_ */