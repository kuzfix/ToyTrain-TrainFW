/*
 * FW_VlakecDuplo.c
 *
 * Created: 09-Dec-18 14:02:02
 * Author : maticpi
 */ 

#include <avr/io.h>
#include <string.h>
#include "config.h"
#include "ADC.h"
#include "systime.h"
#include "UART0_IRQ.h"  //for debug
#include "pdlib_nrf24l01.h"

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

int main(void)
{
  uint32_t now,t1=0,t2=0,t3=0,t4=0,t5=0;
	int status;
	unsigned char address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01};
	char data[] = "12345678901234567890123";

  InitIO();
  Systime_Init();
  ADC_Init();
  DIDR0=(1<<1); //ADC_Init disables input buffers for PC0 - PC3. Only disable the actual analog inputs dig buf
  UART0_Init();
  stdout = &UART0_str;
  Init_Timer1();
  sei();

  NRF24L01_Init();
  NRF24L01_SetAirDataRate(PDLIB_NRF24_DRATE_250KBPS);
  NRF24L01_SetLNAGain(-18); // dBm
  /* Set the address */
  NRF24L01_SetRxAddress(PDLIB_NRF24_PIPE0, address);
  /* Set the packet size */
  NRF24L01_SetRXPacketSize(PDLIB_NRF24_PIPE0, 23);
  NRF24L01_PowerDown();
 /* while (1)
  {
    printf("a");
    _delay_ms(100);
  }*/
  while(!IsBtnPressed()) {} //wait until the button is pressed for the first time
  printf("Main loop:");
  while (1) 
  {
	  if (Has_X_MillisecondsPassed(500,&t5)) 
    {
        printf("S");
		  status = NRF24L01_SendData(data, 23);
	      printf("t");

		  if(PDLIB_NRF24_TX_FIFO_FULL == status)
		  {
			  /* If TX Fifo is full, we can flush it */
	      printf("F");
			  NRF24L01_FlushTX();
	      printf("l");
		  }else if(PDLIB_NRF24_TX_ARC_REACHED == status)
		  {
		      printf("A");
			  while(status == PDLIB_NRF24_TX_ARC_REACHED)
			  {
				  /* Automatic retransmission count reached, we'll attempt the TX continuously until the TX completes */
				  if (Has_X_MillisecondsPassed(100,&t3)) {status = NRF24L01_AttemptTx();}
			    if (Has_X_MillisecondsPassed(1000,&t4)) {printf("!");}
			  }
			  printf("r");
		  }
    }     
    if (Has_X_MillisecondsPassed(1000,&t3)) {printf(".");}

/*
	status = NRF24L01_WaitForDataRx(&pipe);

    if(PDLIB_NRF24_SUCCESS == status)
    {
	  temp = NRF24L01_GetRxDataAmount(pipe);
	  //PrintRegValue("Data Available in: ",pipe);
	  //PrintRegValue("Data amount available : ",temp);
	  memset(data,0x00,32);
	  status = NRF24L01_GetData(pipe, data, &temp);
	  //PrintString("Data Read: ");
	  //PrintString((const char*)data);
	  //PrintString("\n\r");
    }
*/
    if (HasOneMillisecondPassed()) {ProcessQueue();}
    if (Has_X_MillisecondsPassed(100,&t2))  //adjust speed 10x per sec
    {
      now = GetSysTick();
      if (IsBtnPressed()) t1=now;
      if (now - t1 > RUNNING_TIME ) 
      {
        StopTrain();
        
      }        
      else 
      {
        StartTrain();
        
      }       
    }
  }
}
