#include "Arduino.h"
#include "STHardware.h"
#include "STButtons.h"
#include <time.h> 

#include "SMScheduler.h"
#include "STio.h"
extern SMScheduler sched;

void STButtons::setup()
{
    pinMode(PIN_BUTTONS,INPUT);
    pressed=0;
    long_push=0;
    pushed=0;

}

   

int ad_vals[]={BUTTON_LEFT_V,BUTTON_RIGHT_V,BUTTON_TOP_V,BUTTON_BOTTOM_V};
int switch_ids[]={BUTTON_LEFT,BUTTON_RIGHT,BUTTON_TOP,BUTTON_BOTTOM};

#define NUM_BUTTONS 4
#define LONG_PRESS 1000


void STButtons::ProcessInput(void)
{

#ifdef V1
    val = analogRead(PIN_BUTTONS);
    //
    
    //printf("val = %d ",val);
    if (val<4000)
        {
            for (int x=0;x<NUM_BUTTONS;x++)
            if ((val< (ad_vals[x]+BUTTON_RANGE)) && (val>(ad_vals[x]-BUTTON_RANGE)))
                {
                    int id = switch_ids[x];
                    if (pressed!=id)
                        {
                            down_time=sched.clock();//clock();
                            pressed=id;
                        }
                    //printf("Ppp %d %d %d\n",val,id,clock()-Buttons->down_time);
                    if ((sched.clock()-down_time)>LONG_PRESS)
                       long_push=id;
                }
        }
    else
        {
           if (pressed!=0)
            {
              pushed=pressed;
              key_list.push_back(pushed);
              if (key_list.size()>KEY_LIST_LEN)
                 key_list.pop_front();
              pressed =0;
            }
        }
#endif
#ifdef V2
    //vTaskDelay(100);

    uint16_t v = IOReadInput();
    //printf ("V= %x\n",v);
    int id =  (v&0x100)==0 ? BUTTON_LEFT:0;
    id |=    (v&0x200)==0 ? BUTTON_BOTTOM:0;
    id |=    (v&0x400)==0 ? BUTTON_RIGHT:0;
    id |=    (v&0x8000)==0 ? BUTTON_TOP:0;

     if (id)     
        {
                    if (pressed!=id)
                        {
                            down_time=sched.clock();//clock();
                            pressed=id;
                        }
                    printf("[BUT] %x %d\n",id,sched.clock()-down_time);
                    if ((sched.clock()-down_time)>LONG_PRESS)
                       {
                            printf("[BUT] LP  %x %d\n",id,sched.clock()-down_time);
                           long_push=id;
                       }
                
        }
    else
        {
           if (pressed!=0)
            {
              printf("Button released %x \n",v);
              pushed=pressed;
              key_list.push_back(pushed);
              if (key_list.size()>KEY_LIST_LEN)
                 key_list.pop_front();
              pressed =0;
            }
        }
#endif
}

uint8_t STButtons::WaitKey(uint8_t k)
{
    uint8_t p;
    while (1)
     {
         ProcessInput();
         if ((p=GetKey())&k)
            break;
         delay(100);
     }
    return p;
}
uint8_t STButtons::GetKey(void)
{
    uint8_t k = pushed;
    pushed = 0;
    return k;
}
void STButtons::DoTask()
{

    while (1)
    {
    ProcessInput();
    sched.delayRTC(100);
    }

}
void STButtons::ButtonTask(void * parameter)
{
    STButtons *Buttons  = (STButtons *)parameter;
    vTaskDelay(100);
    printf("Starting button task\n");
    Buttons->DoTask();
}