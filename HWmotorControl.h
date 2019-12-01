/*
 * HWmotorControl.h
 *
 * Created: 01/12/2019 12:24:07
 *  Author: maticpi
 */ 


#ifndef HWMOTORCONTROL_H_
#define HWMOTORCONTROL_H_

#include <avr/io.h>

#define NAPREJ  1
#define NAZAJ   0

void Init_Timer1();
void Stop_Timer1();
void Motor(int speed,int direction);

#endif /* HWMOTORCONTROL_H_ */