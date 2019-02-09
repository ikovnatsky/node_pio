
#include "SMScheduler.h"
#define MIN_SLEEP 12
#define MIN_DISP 600
#define MIN_CPU_SLEEP 100
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"

#ifdef INTERNAL_160
#define TOMS(x) (x/160)
#define TORTC(x) (x*160)
#else
//#define TOMS(x) (x/32.768)
//#define TORTC(x) (x*32.768)

#endif

int TOMS( int64_t rtc)
{
  return rtc/32.768;
}
uint64_t TORTC( int ms)
{
  return ms*32.768;
}
int SMScheduler::clock()
{
  return TOMS(rtc_time_get());
}
int SMScheduler::GetTick()
{
  return GetTickRTC();

  return tick;
}
int SMScheduler::GetTickRTC()
{
  uint64_t now;
  now=rtc_time_get();
  now = now-tick_offset;
  return TOMS(now);
}
int SMScheduler::EventExpired()
{
  if (ev_time-GetTick()<=0)
      return 1;
      else
    return 0;
  return ev_expired;
}
int  SMScheduler::EpochStart()
{
   uint64_t now;
   int es;
   now=rtc_time_get();
   now = now-tick_offset;

   es = ((TOMS(now)/2560))*2560;
   cur_ts_low=( (TOMS(now))-es)/10;
   return es;
}
 
int SMScheduler::SetEventTime (int ts_low, int ts_high, int offset_ms)
{
  int sched_time = EpochStart() + (ts_high*256+ts_low)*TICK_PER_TS +offset_ms;
  // lets schedule on next super epoch
  if (sched_time<GetTick()) 
     {
      sched_time = sched_time +(256*super_epoch_size)*TICK_PER_TS;
      //printf("time too late next epoch\n");
     }
  //printf("Schduled current time %d scheduled %d epoch %d\n",GetTick(),sched_time,EpochStart());
  SetEventTime (sched_time);
  return 1;
}

int SMScheduler::WaitUntil(int ts_low, int ts_high, int offset_ms, TaskHandle_t  taskId)
{
  SetEventTime (ts_low,ts_high,offset_ms);
  WaitEventTime(taskId);
  return 1;
}
int SMScheduler::SetEventTime (int ev_tick)
{
  //check
  if (ev_tick<=GetTick())
     {
      
      printf("Too late %d %d  ee %d %d\n",ev_tick,GetTick(),super_epoch_start,EpochStart());
     }
  ev_time=ev_tick;
  ev_sched=1;
  ev_expired=0;
 //  printf("sched at %d %d\n",ev_tick,tick);
  return 1;
}

int SMScheduler::PrintTime(char *txt)
{
  EpochStart(); 
  if (txt==NULL)
     txt="";
  printf("[TIME] %20s %d:%d  tick %d  sched at %d\n",txt,cur_ts_high,cur_ts_low,GetTick(),ev_time);
  return 1;
}

void SMScheduler:: RequestScreenRefresh()
{
  disp_request=1;
}
void SMScheduler:: UpdateDisplay()
{
  disp_now=1;
  //while (disp_now==1)
  //   vTaskDelay(10);
  disp_request=0;
}
#define MAX_SLEEP 100000

void SMScheduler::light_sleep(int time_ms)
{
    if (can_sleep==0)
       {
        printf("Sorry Cant sleep\n");
       return;
       }
    //printf("Enter light sleep\n");
    while (time_ms>0)
    {
    esp_sleep_enable_timer_wakeup(min(time_ms*1000,MAX_SLEEP));
    DoUI();
    wire_take(10);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
    esp_light_sleep_start();
    wire_give();
    time_ms-= min(time_ms,MAX_SLEEP/1000);
    if (can_sleep==0)
       return;
    
    //printf("Timeleft %d\n",time_ms);
    }
    //printf("Done\n");
    
}
void SMScheduler::DoUI(void)
{
// can add check so its not done too often
xSemaphoreGive(user_sem);
xSemaphoreTake(user_sem_done,1); 
}
int SMScheduler::WaitEventTime(TaskHandle_t  taskId)
{
  // should be implemented with an rtos wait

while(1)
{
  //if ( ((ev_time-GetTick())>MIN_DISP) && disp_request)
  //    UpdateDisplay();
  if ( disp_request)
      UpdateDisplay();



  if ((ev_time-GetTick())>MIN_CPU_SLEEP)
      {
        int sleeptime = ev_time-GetTick()-4;
        uint64_t now;
        //int mnow=millis();
        now=rtc_time_get();
        sleeptime=(sleeptime) *1000;
        //printf("light_sleep enter: %d %d\n",sleeptime,clock());
        DoUI();   
        if (can_sleep)
           {
           esp_sleep_enable_timer_wakeup(min(sleeptime,MAX_SLEEP)); 
           taskYIELD();
           wire_take(10);
           esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);

           int ret = esp_light_sleep_start();
           wire_give();
           }
        else
           vTaskDelay(10);
      //  printf("light_sleep wake : %d %d  ticks %d\n", ret,clock(),(int)(rtc_time_get()-now));
       //printf("time %d ev_time %d\n",GE)
       //printf("Request Sleep ret %d %d  actual %d  m%d\n",ret,sleeptime,TOMS((int)(rtc_time_get()-now)),millis()-mnow);
        
       // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);

       // if we have the time let the other tasks do their thing
       continue;
      }
  else
    if ((ev_time-GetTick())>MIN_SLEEP)
       {
         DoUI(); 
         vTaskDelay(ev_time-GetTick()-MIN_SLEEP);
          break;
       }
  break;
   
}
  while(EventExpired()==0);  
  //printf ("Expired at %d  %d  %d %d\n",GetTick(),GetTickRTC(),rtc_clk_fast_freq_get(),rtc_clk_slow_freq_get());
  //printf("------ ee %d %d\n",super_epoch_start,EpochStart());
  // rtc_clk_cpu_freq_set(RTC_CPU_FREQ_240M);
  return 1;
}
void   SMScheduler::ResetTime(int offset_ms)
{
  printf ("offset = %d\n",offset_ms);
 xTimerStop(sTimer,OS_TICK_PER_TICK);
 tick=offset_ms*OS_TICK_PER_TICK;
 tick_offset= rtc_time_get() -TORTC(offset_ms*OS_TICK_PER_TICK);
 cur_ts_low=offset_ms/TICK_PER_TS;
 super_epoch_start = 0;
 cur_ts_high = 0;
 xTimerStart( sTimer, 0 );
}
int  SMScheduler::ScheduleEV( int (*cb)(void *), unsigned int time, void *data)
 {
   int x;
  for (x=0; x<MAX_EVS;x++)
     evs[x].en=0;
  if (x==MAX_EVS)
      return -1;
  evs[x].en=1;
  evs[x].time=time;
  evs[x].data = data;
  evs[x].cb = cb;  
  return x;
 }

/*
int SMScheduler::ScheduleNext()
{
  int best_time=10000;
  int best_ev =0;
  if  (ev_sched)
     return -1;
  for (x=0; x<MAX_EVS;x++)
     if (evs[x].en==1)
       if ((evs[x].time-time)<best_time)
           {
            best_time =(evs[x].time-time);
            best_ev=x;
           } 
           
    ev_sched=1;
    ev_time = evs[best_ev].time;
    ev_cb = evs[best_ev].cb;
    evs[x].en=0;       
}
*/
void   SMScheduler::tickCB()
{
   tick+=1;
   if ((tick%TICK_PER_TS)==0)
      {
        cur_ts_low++;
        if (cur_ts_low==256)
           {
           // Serial.print("Ep");
            cur_ts_low=0;
            cur_ts_high++;
            if (cur_ts_high==super_epoch_size)
                 {
                  cur_ts_high=0;
                  super_epoch_start=tick;
                 }
           }
      }
   if (ev_sched>=0)
      {
        if (GetTick()==ev_time)
           {
            ev_expired=1;
            ev_sched=0;
           }
          
      }
}
SMScheduler *g_sched;

void vTimerCallback( TimerHandle_t xTimer )
 {
   g_sched->tickCB();
 }
 
SMScheduler::SMScheduler()
{
  disp_request=0;
  disp_now=0;
  can_sleep=1;
}

int SMScheduler::Init()
{
    int x;
   cur_ts_high=0;
   cur_ts_low=0;
   tick=0;
   disp_now=0;
   ev_sched=0;
   ev_expired=1;
   super_epoch_size=1;
   super_epoch_start=0;
   g_sched=this;
   for (x=0; x<MAX_EVS;x++)
     evs[x].en=0;
   sTimer=xTimerCreate  ("Timer",1,pdTRUE,( void * ) 0, vTimerCallback);
   xTimerStart( sTimer, 0 );
//   printf("Grea")
   spi_mutex = xSemaphoreCreateMutex();
   user_sem = xSemaphoreCreateBinary();
   user_sem_done = xSemaphoreCreateBinary();
   wire_sem = xSemaphoreCreateMutex();
  // xSemaphoreTake(user_sem,portMAX_DELAY);
  // printf ("mutex = %x\n",(int)spi_mutex);
   return 1;
}


 uint8_t SMScheduler::spi_take(int to, int min_time, char *owner)
 {
  // printf("Take ..");
  int x = 10;
   int err =xSemaphoreTake(spi_mutex,1000);
   if (err==0)
      {
        printf("[SPI]Could not obtain spi lock %s, %s owned by %s\n",owner,pcTaskGetTaskName(xTaskGetCurrentTaskHandle()),spi_owner);

      }
   else
      {
        strcpy(spi_last_owner,spi_owner);
        if(owner) strcpy(spi_owner,owner);
        else
        strcpy(spi_owner,"Dont Know");
        
        spi_take_time=clock();
      }
  // printf("d\n;");

   return 1;
 }
 uint8_t SMScheduler::spi_give()
{
  xSemaphoreGive(spi_mutex);
   return 1;
}

 uint8_t SMScheduler::wire_take(int to, int min_time)
 {
   if (xSemaphoreTake(wire_sem,portMAX_DELAY)==false)
      printf("Semiphore error -------------------->");
   return 1;
 }
 #include "Wire.h"
 uint8_t SMScheduler::wire_give()
{
  Wire.flush();
  xSemaphoreGive(wire_sem);
   return 1;
}
void SMScheduler::delayRTC(int ms)
{
  
  
  int64_t now;
  now=rtc_time_get();
  while ((TOMS(rtc_time_get())-TOMS(now))<ms)
  {
    vTaskDelay(10);
    }
  
}
