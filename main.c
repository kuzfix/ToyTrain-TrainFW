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
#include "TrainControl.h"
#include "nrf24.h"
#include "kbd.h"

#define DEFAULT_AWAKE_TIME (10*60*1000UL)  //10 min

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
/*Listen for 10ms every 500ms*/
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
        //printf("C=%d, data=%s ",counter, data);
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
      if (counter > (500-10-1)) state=0;
      break;
  }
  return result;
}

/* PROTOCOL
Wake - wake up the receiver - no action if already awake
Fxx! - Forward with speed xx: ASCII 0(stop)-99(max speed)
Bxx! - Backwards with speed xx (same format as F)
Txx! - Time to auto stop (xx seconds).
txx! - Time to auto stop if command receifed frome remote controller
sxxx - Time to auto stop if command slow receifed frome remote controller(xx.x seconds)
*/
void DecodeCommand(uint8_t *pu8_cmd)
{
  int8_t num, num10;
  num = pu8_cmd[2] - '0';
  num10 = pu8_cmd[1] - '0';
  //if (param not a number) return;
  if ((num < 0) || (num > 9) || (num10 < 0) || (num10 > 9))
  {
    return;
  }
  
  num = num+10*num10;
  
  if ( (num == 0) && 
      ((pu8_cmd[0] == 'F') || (pu8_cmd[0] == 'B')) )
  {
    TrainStop();
    printf("Stop!");
  }
  else
  {
    switch (pu8_cmd[0])
    {
      case 'F': TrainStart(num, CMD_FORWARD, CMD_SRC_REMOTE); printf(" FWD%d ",num); break;
      case 'B': TrainStart(num, CMD_BACK, CMD_SRC_REMOTE); printf(" BCK%d ",num); break;
      case 'T': SetLocalRunningTime(num); break;
      case 't': SetRemoteCmdRunningTime(num); break;
      case 's': 
        if ((pu8_cmd[0] >= '0') && (pu8_cmd[0] <= '9'))
        {
          SetRemoteCmdSlowRunningTime(num*10 + (pu8_cmd[0]-'0')); 
        }
        break;
    }
  }

}

int main(void)
{
  uint32_t  u32_last_activity_time=0;
  uint8_t   pu8_data[33];
  uint8_t   u8_rxActive=0;
  
  InitIO();
  KBD_Init();
  Systime_Init();
  ADC_Init();
  DIDR0=(1<<1); //ADC_Init disables input buffers for PC0 - PC3. Only disable the actual analog inputs dig buf
  UART0_Init();
  stdout = &UART0_str;
  Init_Timer1();
  InitSleep();
  LoadRunningTimes();
  sei();

  nrf24_init();                   // init hardware pins 
  nrf24_config(2,4);              // Channel #2 , payload length: 4 
  nrf24_tx_address(tx_address);   // Set the device addresses 
  nrf24_rx_address(rx_address);
  nrf24_powerDown();

  printf("Main loop:");
  while (1)
  {
    if (HasOneMillisecondPassed())
    {
      //Background services (kbd, motor control)
      KBD_Read();
      ProcessCommandQueue();
      AutoStop();
      
      // React to buttons
      if (KBD_GetReleasedKey())
      {
        u32_last_activity_time = GetSysTick();
        TrainStart(100, CMD_FORWARD, CMD_SRC_LOCAL);
      }
      
      //React to radio messages
  	  if(!u8_rxActive)
      {
        u8_rxActive = CheckForCarrier();
        if (u8_rxActive) u32_last_activity_time = GetSysTick();
      }      
      else //Active listening
  	  {
        if(nrf24_dataReady())
        {
          memset(pu8_data,0x00,33);
          nrf24_getData(pu8_data);
//          nrf24_powerDown();  //Powering down dramatically reduces reception reliability
//          u8_rxActive=0;
          DecodeCommand(pu8_data);
          u32_last_activity_time = GetSysTick();
        }
  	  }

///////////// Deep sleep activation //////////////////////
      if (GetSysTick() - u32_last_activity_time > DEFAULT_AWAKE_TIME)
      {
        nrf24_powerDown();
        u8_rxActive=0;
        printf("Going to sleep..."); _delay_ms(50); //Delay to allow to finish transmission.
        GoToSleep();
        //Just woke up - Start the train (The only way to do it is with the local button)
        printf("\r\nAwake!");
        TrainStart(100, CMD_FORWARD, CMD_SRC_LOCAL);
        u32_last_activity_time = GetSysTick();
      }
    }     
  }
}
