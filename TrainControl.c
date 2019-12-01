/*
 * TrainControl.c
 *
 * Created: 01/12/2019 12:12:45
 *  Author: maticpi
 */ 

#include "TrainControl.h"
#include "HWmotorControl.h"

#define QUEUE_LENGTH  10

int gi_currentTrainSpeed;
uint8_t commandQueue[QUEUE_LENGTH];
int16_t commandQueueParam[QUEUE_LENGTH];
uint32_t commandQueueTime[QUEUE_LENGTH];

int IsCommandQueueClear();
int InsertCommand(uint8_t cmd, int16_t param, uint32_t timestamp);

void StopTrain()
{
  uint32_t now;
  if (gi_currentTrainSpeed)
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
      gi_currentTrainSpeed=0;
      printf("t");
    }
  }
}

void TrainForward(int speed)
{
  uint32_t now;
  
  if (speed > 98) speed = 100;
  if (speed < 0)  speed = 0;
  
  if ((speed > gi_currentTrainSpeed))
  {
    if (IsCommandQueueClear())
    {
      now = GetSysTick();
      InsertCommand(CMD_FORWARD,speed,now);
      InsertCommand(CMD_SND_TRG_START,0,now);
      InsertCommand(CMD_SND_TRG_STOP,0,now+100);
      gi_currentTrainSpeed=speed;
      printf("S");
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

void ProcessCommandQueue()
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
