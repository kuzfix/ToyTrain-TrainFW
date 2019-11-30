/*
 * FW_VlakecDuplo.c
 *
 * Created: 09-Dec-18 14:02:02
 * Author : maticpi
 */ 

#include <avr/io.h>
#include <string.h>
#include <avr/sleep.h>
#include "config.h"
#include "ADC.h"
#include "systime.h"
#include "UART0_IRQ.h"  //for debug
//#include "pdlib_nrf24l01.h"
#include "nrf24.h"

#define CARRIER_DETECTED  1

//#define AWAKE_TIME (10*60*1000UL)  //10 min
#define AWAKE_TIME (100*60*1000UL)  //100 min

#define RUNNING_TIME  5000UL
//#define IsBtnPressed() (PIND & (1<<2))
#define IsBtnPressed() (PINC & (1<<2))
#define NAPREJ  1
#define NAZAJ   0
#define SndTrgOn() {DDRD |= (1<<6); PORTD |= (1<<6); DbgOn();}     //Has to be "1" 
#define SndTrgOff() {DDRD &= ~(1<<6); PORTD &= ~(1<<6); DbgOff();}  //          or "HZ" (no pull-up)
#define DbgOn() PORTD |= (1<<5)
#define DbgOff() PORTD &= ~(1<<5)

void InitIO()
{
  PORTE=0xFF;
  PORTC=0xF9; //PC1 = ADC power sense, PC2 = BTN (pulldown, short to VCC)
  PORTD=0x4F; //PD7 = CE(NRF24..), PD2=IRQ, PD1=TX,PD0=RX, PD5=DbgLED, PD6=start/stop sound
  PORTB=0x11; //PB0 = nCS, PB1,2=motA,B, PB3=MOSI, PB4=MISO, PB5=SCK (PB6,7=XTAL)
  //1-out, 0-in
  DDRE=0x00;
  DDRC=0x00;
  DDRD=0xa2;
  DDRB=0x2F;
}

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

#define CMD_FORWARD           1
#define CMD_BACK              2
#define CMD_SND_TRG_START     3
#define CMD_SND_TRG_STOP      4
#define CMD_BLOCK             5
#define CMD_PENDING_BITMASK 0x80

#define QUEUE_LENGTH  10
uint8_t commandQueue[QUEUE_LENGTH];
int16_t commandQueueParam[QUEUE_LENGTH];
uint32_t commandQueueTime[QUEUE_LENGTH];

int InsertCommand(uint8_t cmd, int16_t param, uint32_t timestamp)
{
  int result=0;
  int i;
  //find empty place in queue
  for (i=0; i<QUEUE_LENGTH; i++)
  {
    if (!(commandQueue[i] & CMD_PENDING_BITMASK)) // if (slot free)
    {
      commandQueue[i] = cmd | CMD_PENDING_BITMASK;
      commandQueueParam[i] = param;
      commandQueueTime[i] = timestamp;
       printf("\nIn %d:c=%02X, p=%d, t=%ld",i,commandQueue[i],commandQueueParam[i],commandQueueTime[i]);
      result=1;
      break; 
    }
  }
  return result;
}

void ProcessQueue()
{
  int i;
  uint8_t cmd;
  int16_t param;
  uint32_t now = GetSysTick();
  
  for (i=0; i<QUEUE_LENGTH; i++)
  {
    if (commandQueue[i] & CMD_PENDING_BITMASK)
    {
      if (now >= commandQueueTime[i] ) //if (pending cmd and the time is right)
      {
        cmd = commandQueue[i] & (~CMD_PENDING_BITMASK); //get command
        param = commandQueueParam[i];
        commandQueue[i]=0;                              //delete command from queue
        printf("\nOut%d: c=%02X p=%d, t=%ld, CQ=0x%02X", i, cmd, param, commandQueueTime[i], commandQueue[i]);
        switch (cmd)
        {
          case CMD_FORWARD:
            Motor(param,NAPREJ);
            break;
          case CMD_BACK:
            Motor(param,NAZAJ);
            break;
          case CMD_SND_TRG_START:
            SndTrgOn();
            break;
          case CMD_SND_TRG_STOP:
            SndTrgOff();
            break;
          case CMD_BLOCK:
            break;
          default:  //speed command or unknown command
          //Report error
            while (1)
            {
              DbgOn();
              _delay_ms(100);
              DbgOff();
              _delay_ms(100);
            }
        }
      }
    }
  }
}

int IsCommandQueueClear()
{
  int i;
  for (i=0; i<QUEUE_LENGTH; i++)
  {
    if (commandQueue[i] & CMD_PENDING_BITMASK) return 0;
  }
  return 1;  
}

int trainStarted;

void StopTrain()
{
  uint32_t now;
  if (trainStarted)
  {
    if (IsCommandQueueClear())
    {
      now = GetSysTick();
      InsertCommand(CMD_SND_TRG_START,0,now);
      InsertCommand(CMD_SND_TRG_STOP,0,now+100);
      InsertCommand(CMD_FORWARD,90,now);
      InsertCommand(CMD_FORWARD,80,now+100);
      InsertCommand(CMD_FORWARD,70,now+200);
      InsertCommand(CMD_FORWARD,50,now+300);
      InsertCommand(CMD_FORWARD,30,now+400);
      InsertCommand(CMD_FORWARD,0,now+500);
      trainStarted=0;
      printf("t");
    }
  }
}

void StartTrain()
{
  uint32_t now;
  if (!trainStarted)
  {
    if (IsCommandQueueClear())
    {
      now = GetSysTick();
      InsertCommand(CMD_FORWARD,100,now);
      InsertCommand(CMD_SND_TRG_START,0,now);
      InsertCommand(CMD_SND_TRG_STOP,0,now+100);
      trainStarted=1;
      printf("S");
    }
  }      
}

void InitSleep()
{
  //Init pin change interrupt for pin PC2 (PinChInt 1) 
  //PCICR |= (1<<PCIE1);
  PCMSK1 |= (1<<PCINT10);
  //Init sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void GoToSleep()
{
  //TODO: Check the datasheet on all the details on how to conserve more power
  Stop_Timer1();
  sleep_enable();
  PCIFR = (1<<PCIF1);
  PCICR |= (1<<PCIE1);
  sleep_cpu();
}

ISR(PCINT1_vect)
{
  sleep_disable();
  PCICR &= ~(1<<PCIE1); //disable this interrupt 
  Init_Timer1();
}

	uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
	uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};

/*call exactly once per millisecond*/
/*Listen for 1ms every 100ms*/
int CheckForCarrier()
{
  static int state=0;
  static unsigned int counter;
  uint8_t data[33];
  int result = 0;
      
  switch (state)
  {
    case 0: //Enable RxMode
      nrf24_powerUpRx();
      counter=0;
      state++;
      break;
    case 1: //Wait 10 cycles for wakeup call
      if(nrf24_dataReady())
      {
        //Wakeup call received        
    		memset(data,0x00,33);
        nrf24_getData(data);
        //TODO: check msg if it really is a wakeup call
        printf("C=%d, data=%s ",counter, data);
        result = 1;
        counter = 0;
        state = 0;
      }
      else 
      {
        counter++;
        if (counter > 10) state++;
      }      
      break;
    case 2: //No wakeup call received
      printf("n");
      nrf24_powerDown();
      counter=0;
      state++;
      break;
    default:
      counter++;
      if (counter > 500) state=0;
      break;
  }
  return result;
}
/*
int ListenForMessages()
{
  return 0;
}*/
int main(void)
{
  uint8_t data[33];
  int status=0;
  
  InitIO();
  Systime_Init();
  ADC_Init();
  DIDR0=(1<<1); //ADC_Init disables input buffers for PC0 - PC3. Only disable the actual analog inputs dig buf
  UART0_Init();
  stdout = &UART0_str;
  Init_Timer1();
  InitSleep();
  sei();

  printf("Starting:\r\n");
  
  nrf24_init();                   // init hardware pins 
  nrf24_config(2,4);              // Channel #2 , payload length: 4 
  nrf24_tx_address(tx_address);   // Set the device addresses 
  nrf24_rx_address(rx_address);
  
  nrf24_powerDown();

  while (1)
  {
    /*if(nrf24_dataReady())
    {
   		memset(data,0x00,33);
      nrf24_getData(data);
      printf("> %s ",data);
    }*/
    if (HasOneMillisecondPassed())
    {
  	  if(!status)
        status = CheckForCarrier();
      else
  	  {
        if(nrf24_dataReady())
        {
          memset(data,0x00,33);
          nrf24_getData(data);
          printf("> %s ",data);
        }
        status++;
        if (status==0) nrf24_powerDown();
  	  }
    }     
  }
}
 /*
  while(!IsBtnPressed()) {} //wait until the button is pressed for the first time
  printf("Main loop:");
  while (1) 
  {
    if (HasOneMillisecondPassed())
    {
      ProcessQueue();
      if (iActiveRXmode == 0) //Passive listening - just checking for carrier periodically
      {
        if (CheckForCarrier() == CARRIER_DETECTED) 
        {
          iActiveRXmode = 1;
          last_activity_time = GetSysTick();
        }          
      }
      else //Active listening
      {
        if (ListenForMessages())
        {
          last_activity_time = GetSysTick();
        }
      }
      if (GetSysTick() - last_activity_time > AWAKE_TIME)
      {
        NRF24L01_PowerDown();
        printf("Going to sleep...");
        GoToSleep();
        //Just woke up - Start the train 
        printf(" Awake");
        StartTrain();
        t1 = GetSysTick();
        last_activity_time = t1;
      }
    }
    if (Has_X_MillisecondsPassed(100,&t2))  //adjust speed 10x per sec
    {
      now = GetSysTick();
      if (IsBtnPressed()) 
      {
        t1=now;
        last_activity_time = now;
      }        
      if (now - t1 > RUNNING_TIME ) 
      {
        StopTrain();
      }        
      else 
      {
        StartTrain();
      }       
    }
    if (Has_X_MillisecondsPassed(1000,&t3)) {printf(".");}
  }
}
*/


