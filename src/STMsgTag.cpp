
#include "Arduino.h"

#define TAG_BLE_SHORT      1
#define TAG_TIME           2
#define TAG_GPS_POS_LONG   0x1c
#define TAG_BATTERY        4
#define TAG_BLE_LONG       5
#define TAG_LORA_HF        6
#define TAG_SOC            7
#define TAG_PANIC          8
#define TAG_PRESSURE       9



#define HPA_MIN 800
#define HPA_MAX 1200#define MAX_TAG_MSG 20
class STMsgTag
{
    public:
      STMsgTag(){len=0;};
      uint8_t msg[MAX_TAG_MSG];
      uint8_t len;
      uint8_t priority;
      uint8_t Len(){return len;};
   
};

class STMsgGPS : public STMsgTag
{
    STMsgGPS ( ){len =15;};
    STMsgGPS ( float lon, float lat, float z, short dop)
       {
        msg[0]= TAG_GPS_POS_LONG;
        memcpy(&msg[1],&lon,4);
        memcpy(&msg[1+4],&lat,4);
        memcpy(&msg[1+8],&z,4);
        memcpy(&msg[1+12],&dop,2);
        len= 15;
        }
};

class STMsgLORA_HF : public STMsgTag
{
    STMsgLORA_HF ( ){len=5;};
    STMsgLORA_HF ( short id, short dist)
        {
        msg[0]= TAG_LORA_HF;
        msg[1] = id&0xff;
        msg[2]= (id >>8)&0xff;
        memcpy(&msg[1+2],&dist,2);
        len= 5;
        }
};

class STMsgPanic : public STMsgTag
{
    STMsgPanic(){msg[0]=TAG_PANIC;len=1;};
};

class STMsgPressure : public STMsgTag
{
    public:
        STMsgPressure(){len=3;};
        STMsgPressure ( float hPa)
                {
            	unsigned short val;
	            val = (hPa-HPA_MIN)*65000.0/(HPA_MAX-HPA_MIN);
                msg[0]=TAG_PRESSURE;
                msg[1] = val&0xff;
                msg[2] = (val>>8)*0xff;
                len=3;
                }
}

#include <iostream> 
#include <list> 
#include <iterator> 
using namespace std; 
void TestMe()
{
    list <STMsgTag> list;

    STMsgPressure p(10);
    list.push_back(p);

}