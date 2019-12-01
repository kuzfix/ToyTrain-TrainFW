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

void TrainStop()
{
  uint32_t now,nowinc;
  if (gi_currentTrainSpeed != 0)
  {
    if (IsCommandQueueClear())
    {
      now = GetSysTick();
      InsertCommand(CMD_SND_TRG_START,0,now);
      InsertCommand(CMD_SND_TRG_STOP,0,now+100);
      nowinc=now-100;
      //if going forward
      if (gi_currentTrainSpeed > 90) InsertCommand(CMD_FORWARD,90,nowinc+=100);
      if (gi_currentTrainSpeed > 80) InsertCommand(CMD_FORWARD,80,nowinc+=100);
      if (gi_currentTrainSpeed > 70) InsertCommand(CMD_FORWARD,70,nowinc+=100);
      if (gi_currentTrainSpeed > 50) InsertCommand(CMD_FORWARD,50,nowinc+=100);
      if (gi_currentTrainSpeed > 30) InsertCommand(CMD_FORWARD,30,nowinc+=100);
      //if going backwards
      if (gi_currentTrainSpeed < -90) InsertCommand(CMD_BACK,90,nowinc+=100);
      if (gi_currentTrainSpeed < -80) InsertCommand(CMD_BACK,80,nowinc+=100);
      if (gi_currentTrainSpeed < -70) InsertCommand(CMD_BACK,70,nowinc+=100);
      if (gi_currentTrainSpeed < -50) InsertCommand(CMD_BACK,50,nowinc+=100);
      if (gi_currentTrainSpeed < -30) InsertCommand(CMD_BACK,30,nowinc+=100);
     
      InsertCommand(CMD_FORWARD,0,nowinc);
      gi_currentTrainSpeed=0;
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
    }
  }      
}

void TrainBackwards(int speed)
{
  uint32_t now;
  
  if (speed > 98) speed = 100;
  if (speed < 0)  speed = 0;
  
  if ((-speed < gi_currentTrainSpeed) && (gi_currentTrainSpeed <= 0))
  {
    if (IsCommandQueueClear())
    {
      now = GetSysTick();
      InsertCommand(CMD_BACK,speed,now);
      InsertCommand(CMD_SND_TRG_START,0,now);
      InsertCommand(CMD_SND_TRG_STOP,0,now+100);
      gi_currentTrainSpeed=-speed;
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
            printf("Unknown command: %d (%s,L:%d, %s)",cmd,__FILE__,__LINE__,__FUNCTION__);
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
       //printf("\nIn %d:c=%02X, p=%d, t=%ld",i,commandQueue[i],commandQueueParam[i],commandQueueTime[i]);
      result=1;
      break; 
    }
  }
  return result;
}
