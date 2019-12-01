/*
 * HWmotorControl.c
 *
 * Created: 01/12/2019 12:23:50
 *  Author: maticpi
 */ 
#include "HWmotorControl.h"

void Init_Timer1()  //for PWM motor control 
{
  OCR1A=0xFFFF; //Output=0
  OCR1B=0xFFFF; //Output=0
  TCCR1A=(3<<COM1A0) | (3<<COM1B0) | (3<<WGM10); //fast 10bit PWM, inverting mode (allows constant 0 output)
  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM12) | (1<<CS10); //no prescaller 
}

void Stop_Timer1()
{
  TCCR1A=0;
  TCCR1B=0;
  PORTB &=~(3<<1);  //Turn off motor
}

void Motor(int speed,int direction)
{
  uint16_t pwm_val;
  
  if (speed > 100) speed = 100;
  if (speed < 0) speed = 0;
  
  pwm_val = 1023 - (1023L*speed/100);
  
  if (speed == 0)
  {
    OCR1A=0xFFFF;
    OCR1B=0xFFFF;
  }    
  else if (direction == NAPREJ)
  {
    OCR1A=0xFFFF;
    OCR1B=pwm_val;
  }
  else
  {
    OCR1B=0xFFFF;
    OCR1A=pwm_val;
  }
}