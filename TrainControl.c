/*
 * TrainControl.c
 *
 * Created: 01/12/2019 12:12:45
 *  Author: maticpi
 */ 

#include "TrainControl.h"
#include "HWmotorControl.h"

int gi_currentTrainSpeed;
uint32_t  gu32_motorStartTime=0;

uint32_t  gu32_runningTime=DEFAULT_RUNNING_TIME;
uint32_t  gu32_localRunningTime=DEFAULT_RUNNING_TIME;
uint32_t  gu32_remoteCmdRunningTime=DEFAULT_RUNNING_TIME;
uint32_t  gu32_remoteCmdSlowRunningTime=DEFAULT_SLOW_RUNNING_TIME;

//This is more of a command heap then queue - command execution order does not depend
// on command insertion order
uint8_t commandQueue[QUEUE_LENGTH];
int16_t commandQueueParam[QUEUE_LENGTH];
uint32_t commandQueueTime[QUEUE_LENGTH];

int IsCommandQueueClear();
int InsertCommand(uint8_t cmd, int16_t param, uint32_t timestamp);

void AutoStop()
{
  uint32_t u32_now;
  u32_now = GetSysTick();
  if (u32_now - gu32_motorStartTime > gu32_runningTime )
  {
    TrainStop();
  }
}

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

void TrainStart(int speed, int direction, int src)
{
  int direction_sign; //1 (forward) or -1
  int cmdValid=0;
  uint32_t now=GetSysTick();
  uint32_t nowinc; 
  
  if (speed > 98) speed = 100;
  if (speed < 0)  speed = 0;

  if (direction == CMD_FORWARD) direction_sign = 1;
  else if (direction == CMD_BACK) direction_sign =-1;
  else return;
  
  if (speed*direction_sign == gi_currentTrainSpeed)
  {
    cmdValid=1;
  }
  else if (IsCommandQueueClear())
  {
    if (gi_currentTrainSpeed == 0)
    {
      //Start moving
      InsertCommand(CMD_SND_TRG_START,0,now);
      InsertCommand(CMD_SND_TRG_STOP,0,now+100);
       InsertCommand(direction,100,now);  //Start pulse
      nowinc=now+10;
      for (int spd=10; spd<speed; spd+=10, nowinc+=100)
      {
        InsertCommand(direction,spd,nowinc); 
      }          
      InsertCommand(direction,speed,nowinc);
      gi_currentTrainSpeed=speed*direction_sign;
      cmdValid=1; 
    }
    else if ( ((direction == CMD_FORWARD) && (gi_currentTrainSpeed > 0)) ||
              ((direction == CMD_BACK   ) && (gi_currentTrainSpeed < 0))  )
    { 
      //if no direction change required
      if ( ((direction == CMD_FORWARD) && (speed > gi_currentTrainSpeed)) ||
           ((direction == CMD_BACK   ) && (speed >-gi_currentTrainSpeed))  )
      {  
        //Accelerate
        nowinc=now;
        for (int spd=gi_currentTrainSpeed*direction_sign; spd<speed; spd+=10, nowinc+=100)
        {
          InsertCommand(direction,spd,nowinc);
        }
        InsertCommand(direction,speed,nowinc);
        gi_currentTrainSpeed=speed*direction_sign;
        cmdValid=1;  
      }
      else if ( ((direction == CMD_FORWARD) && (speed <  gi_currentTrainSpeed)) ||
                ((direction == CMD_BACK   ) && (speed < -gi_currentTrainSpeed)))
      {
        //Decelerate
        nowinc=now;
        for (int spd=gi_currentTrainSpeed*direction_sign; spd>speed; spd-=10, nowinc+=100)
        {
          InsertCommand(direction,spd,nowinc);
        }
        InsertCommand(direction,speed,nowinc);
        gi_currentTrainSpeed=speed*direction_sign;
        cmdValid=1;
      }
          
    }
    else  //if direction change required, stop instead
    { 
      TrainStop();
    }

  }    
  //Update running time (depending on source and speed)
  if (cmdValid)
  {
    gu32_motorStartTime=now;
    if (src == CMD_SRC_LOCAL) gu32_runningTime=gu32_localRunningTime;
    else
    {
      if (speed <= 50)
      gu32_runningTime=gu32_remoteCmdSlowRunningTime;
      else
      gu32_runningTime=gu32_remoteCmdRunningTime;
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
      printf(" %c=%d",((commandQueue[i] & ~CMD_PENDING_BITMASK) == CMD_FORWARD) ? 'F':'B',commandQueueParam[i]);
      result=1;
      break; 
    }
  }
  if(!result) printf(" QFull");
  return result;
}

void SetLocalRunningTime(int t)
{
  if (t>0 && t<100)
  {
    gu32_localRunningTime=t*1000UL;
    eeprom_write_byte((uint8_t*)0, t);
    printf(" Set time to autostop: %ld ms",gu32_localRunningTime);
  }
}

void SetRemoteCmdRunningTime(int t)
{
  if (t>0 && t<100)
  {
    gu32_remoteCmdRunningTime=t*1000UL;
    eeprom_write_byte((uint8_t*)1, t);
    printf(" Set time to autostop (remote): %ld ms",gu32_remoteCmdRunningTime); 
  }
}

void SetRemoteCmdSlowRunningTime(int t)
{
  if (t>0 && t<1000)
  {
    gu32_remoteCmdSlowRunningTime=t*100UL;
    eeprom_write_word((uint16_t*)2, t);
    printf(" Set time to autostop (remote slow): %ld ms", gu32_remoteCmdSlowRunningTime); 
  }
}

void LoadRunningTimes()
{
  int t;
  t=eeprom_read_byte((uint8_t*)0);
  if (t>0 && t<100)
  {
    gu32_localRunningTime=t*1000UL;
    printf(" Stored time to autostop: %ld ms",gu32_localRunningTime);
  }

  t=eeprom_read_byte((uint8_t*)1);
  if (t>0 && t<100)
  {
    gu32_remoteCmdRunningTime=t*1000UL;
    printf(" Stored time to autostop (remote): %ld ms",gu32_remoteCmdRunningTime);
  }

  t=eeprom_read_word((uint16_t*)2);
  if (t>0 && t<1000)
  {
    gu32_remoteCmdSlowRunningTime=t*100UL;
    printf(" Stored time to autostop (remote slow): %ld ms", gu32_remoteCmdSlowRunningTime);
  }
}