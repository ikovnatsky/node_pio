#include "Arduino.h"
#include "STHardware.h"
#include "STButtons.h"
#include <time.h> 

#include "SMScheduler.h"
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
      
    //vTaskDelay(100);

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