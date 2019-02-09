
//#include <TinyGPS++.h>
#include "/home/me/Documents/PlatformIO/Projects/node_pio/lib/TinyGPSPlus-1.0.2/src/TinyGPS++.h"

#include "STHardware.h"

class STGps : public TinyGPSPlus 
{
 public:
 void setup();
 void displayInfo();
 void GPSTest();
 void Enable(uint8_t en);
 static void GPSTask( void * param);
 void Process();
};
