#include <Arduino.h>

#include "HardwareSerial.h"
//#include "TinyGPS++.h"
#include "/home/me/Documents/PlatformIO/Projects/node_pio/lib/TinyGPSPlus-1.0.2/src/TinyGPS++.h"

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
  if (en)
    MySerial.print("$PMTK161,1*29\r\n");//$PMTK225,0*2B
else 
    MySerial.print("$PMTK161,0*28\r\n");//$PMTK225,0*2B

}
void STGps::GPSTask(void * parameter)
{
  STGps *p_gps = (STGps *)parameter;
  printf("Starting gps task \n");
  //while (1)
   vTaskDelay(100);
  MySerial.print("$PMTK225,8*23");//$PMTK225,0*2B
  MySerial.print("\r\n");
  p_gps->GPSTest();
}

void STGps::Process()
{
     int b;
    int x=0;

    while ((b = MySerial.read())>=0)
        {
          encode(b);
          printf("%c",b);
        }
    b = Serial.read();
    if (b >= 0)
      MySerial.write(b);
    //{
     // Serial.write(b);
     //     if (encode(b))
     //       displayInfo();
    //}
    //b = Serial.read();
    //if (b >= 0)
    //  MySerial.write(b);
  
}
void STGps::GPSTest()
{
 

   int b;

  while (1)
  {
    int x=0;
    b = MySerial.read();
    if (b >= 0)
        {
          encode(b);
        }
    else
       vTaskDelay(10);
  }
}