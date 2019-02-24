#include <Arduino.h>

#include "HardwareSerial.h"
//#include "TinyGPS++.h"
#include "/home/me/Documents/PlatformIO/Projects/node_pio/lib/TinyGPSPlus-1.0.2/src/TinyGPS++.h"

#include "SMScheduler.h"
extern SMScheduler sched;

HardwareSerial MySerial(1);

#include "STGps.h"


void STGps::setup( )
{
 MySerial.begin(9600,SERIAL_8N1,GPS_RX_PIN,GPS_TX_PIN);
}
void STGps::displayInfo()
{
  Serial.print(F("\nLocation: "));
  // if (gps.location.isValid())
  {
    Serial.print(location.lat(), 6);
    Serial.print(F(","));
    Serial.print(location.lng(), 6);
  }
  // else
  // {
  //   Serial.print(F("INVALID"));
  // }

  Serial.print(F("  Date/Time: "));
  if (date.isValid())
  {
    Serial.print(date.month());
    Serial.print(F("/"));
    Serial.print(date.day());
    Serial.print(F("/"));
    Serial.print(date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (time.isValid())
  {
    if (time.hour() < 10)
      Serial.print(F("0"));
    Serial.print(time.hour());
    Serial.print(F(":"));
    if (time.minute() < 10)
      Serial.print(F("0"));
    Serial.print(time.minute());
    Serial.print(F(":"));
    if (time.second() < 10)
      Serial.print(F("0"));
    Serial.print(time.second());
    Serial.print(F("."));
    if (time.centisecond() < 10)
      Serial.print(F("0"));
    Serial.print(time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }
  Serial.print("Sats:");

  Serial.print(satellites.value()); // Number of satellites in use (u32)
  Serial.print("Hdop:");

  Serial.print(hdop.value());
  Serial.println();
}

void STGps::Enable(uint8_t en)
{
  // if we issued a wakeup to the gps dont change the power settings
  if (did_wake_up)
    {
      // printf ("Waiting for gps dont sleep or wake  %d\n",sched.clock()-wake_time);
      return;
    }
  if (en)
    MySerial.print("$PMTK161,1*29\r\n");//$PMTK225,0*2B
  else 
    MySerial.print("$PMTK161,0*28\r\n");//$PMTK225,0*2B
    state=en;

}
void STGps::GPSTask(void * parameter)
{
  STGps *p_gps = (STGps *)parameter;
  printf("Starting gps task \n");
  //while (1)
   vTaskDelay(100);
  //MySerial.print("$PMTK225,8*23");//
  //MySerial.print("$PMTK225,0*2B");
 // MySerial.print("\r\n");
  MySerial.print("$PMTK104*37");
   MySerial.print("\r\n");
 //MySerial.print("\r\n");
  p_gps->GPSTest();
 
}

uint8_t STGps::GPSSending()
{
  return (clock()-last_rx)<2;
}
#define MAX_AGE (60000)

uint8_t STGps::Process()
{
    int b;
    int x=0;
   // printf(".");
   static int div;

   if (div++>500)
     {
       printf ("GPS age = %d sched %d dw %d\n",location.age(),sched.in_wait,did_wake_up);
       div=0;
     }

    while ((b = MySerial.read())>=0)
        {
          encode(b);
          //if (did_wake_up)
          //   printf("%c",b);
          last_rx=clock();
        }
    if ((location.age()>MAX_AGE || (location.isValid()==0) )&&(did_wake_up==0))
       {
        //MySerial.print("$PMTK225,8*23");//$PMTK225,0*2B
        //Serial.print("\r\n");
        //Enable(1);
        printf("GPS Waking UP\n");
        did_wake_up=1;
        state=1;
        wake_time= sched.clock();
        if (no_sleep_requested==0)
          {
            sched.RequestNoSleep("GPS",0x4);
            no_sleep_requested=1;
          }

       }

    // if we woke up the gps and got a new location we are done
    if (did_wake_up && (location.age()<MAX_AGE) && (location.age()>0) && (location.isValid()))
        {
         did_wake_up=0;
        if (no_sleep_requested)
          {
            sched.ReleaseNoSleep(0x4);
            no_sleep_requested=0;
          }
        //Enable(0);
        printf("GPS can sleep UP\n");
        }

   /* if (did_wake_up && ((sched.clock()-wake_time)>MAX_AGE))
       {
         printf("gps did not wake up lets try agiain\n");
         did_wake_up=0;
       }*/
  return 1;
}


void STGps::GPSTest()
{
 

   int b;

  Enable(1);
  did_wake_up=1;
  sched.RequestNoSleep("GPS",0x4);
  no_sleep_requested=1;
  while (1)
  {
    int x=0;
    Process();
    vTaskDelay(10);
  }
}   