#include <Arduino.h>
#include "freertos/FreeRTOS.h"

typedef int BEACON_TIME  ;

#define TICK_PER_TS 10
#define OS_TICK_PER_TICK 1
typedef struct 
{
   int time;
  void *data;
  int (*cb)(void *);
  char en;
}SchedEvent;


#define MAX_EVS 2
class  SMScheduler
{
public:
 SMScheduler();
 int  Init();
 void ResetTime(int offset_ms);
 int  GetTick();
 int  GetTickRTC();
 int  EpochStart();
 void delayRTC(int ms);
 int clock();
 int  cur_ts_low;
 int  cur_ts_high;
 int  super_epoch_size;
 char spi_owner[20];
 char spi_last_owner[20];
 int spi_take_time;

 uint64_t tick_offset;
 uint8_t spi_take(int a=0, int b=0, char *owner=0);
 uint8_t spi_give(void);
 void   tickCB();
 TimerHandle_t sTimer;
  int tick;
  int super_epoch_start;

 int SetEventTime (int ts_low, int ts_high, int offset_ms);
 int SetEventTime (int ev_tick);
 int WaitEventTime(TaskHandle_t  taskId);
 int WaitUntil(int ts_low, int ts_high, int offset_ms, TaskHandle_t  taskId=0);
 int EventExpired();
 void UpdateDisplay();
 void light_sleep(int time_ms);
 void DoUI(void);
 void RequestScreenRefresh();
 
 int  ScheduleEV(int (*cb)(void *), unsigned int time, void *data);
 SchedEvent evs[MAX_EVS];
  int ev_time;
 int ev_sched;
 volatile int ev_expired;
 int (*ev_cb)(void *);
 
 SemaphoreHandle_t spi_mutex;
 SemaphoreHandle_t user_sem;
 SemaphoreHandle_t user_sem_done;
 SemaphoreHandle_t wire_sem;


 
 uint8_t wire_take(int to, int min_time=0);
 uint8_t wire_give();
 
 int PrintTime(char *txt=NULL);
 uint8_t disp_request;
 uint8_t disp_now;
 uint8_t can_sleep;
};
