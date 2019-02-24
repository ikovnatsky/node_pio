#include "SparkFunIMU.h"
#include "SparkFunLSM303C.h"
#include "LSM303CTypes.h"


#include "SMScheduler.h"
#include "STMotion.h"
extern SMScheduler sched;

#include <math.h>
#include <float.h>

uint8_t isnormal(float v)
{
     if (isnan(v))
         return 0;
     if (isinf(v))
         return 0;
     return 1;
}
void STMotion:: setup()
{
  if (myIMU.begin() != IMU_SUCCESS)
  {
    Serial.println("Failed setup.");
    while(1);
  }
   sched.wire_take(10);
   acc_avg[0]=myIMU.readAccelX();
   acc_avg[1]=myIMU.readAccelY();
   acc_avg[2]=myIMU.readAccelZ();
   sched.wire_give();
}

void STMotion ::Process()
{
   sched.wire_take(10);
   float v;
   v = myIMU.readAccelX();
   if (isnormal(v))
        acc_vec[0]=v;
   v = myIMU.readAccelY();
   if (isnormal(v))
        acc_vec[1]=v;
   v = myIMU.readAccelZ();
   if (isnormal(v))
        acc_vec[2]=v;

   sched.wire_give();
  
  for (int x=0;x <3;x++)
        acc_avg[x] = acc_avg[x]*.7+acc_vec[x]*.3;
   vsum=0;
  for (int v=0;v<3;v++)
               vsum+=(acc_avg[v]-acc_vec[v])*(acc_avg[v]-acc_vec[v]);
   vsum=(float)sqrt((float)vsum);
   if (isnormal(vsum)==0)
      printf("Vsum is nuts %3.3f %3.3f %3.3f\n",acc_avg[0],acc_avg[1],acc_avg[2]);
   if (vsum>th)
     state=STATE_WALKING;
   else
     state=STATE_STILL;
}

uint8_t STMotion ::IsActive(void)
{
  if (state==STATE_WALKING)
   return 1;
  else
   return 0;
}