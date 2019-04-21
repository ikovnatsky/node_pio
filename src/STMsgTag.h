
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
#define TAG_SET_PARAM     10
#define TAG_GET_PORAM     11
#define TAG_ALERT         12
#define TAG_SET_RESP      14
#define TAG_MOTION        15
#define TAG_GPS_STATION   16


#define PARAM_LINE1      1
#define PARAM_LINE2      2
#define PARAM_SSID       3
#define PARAM_PSWD       4


#define HPA_MIN 800
#define HPA_MAX 1200
#define MAX_TAG_MSG 20

class TagSendTimer
{
    public:
      TagSendTimer(){sendPeriod=20000;};
      TagSendTimer(int cur_time){sendPeriod=0;sent_time=cur_time;};
    
    void SetPeriod(int time_ms){sendPeriod=time_ms;};
    uint8_t SendNow(int cur_time, uint8_t auto_reset = true){
        if (((cur_time-sent_time) >sendPeriod) || first)
           {
               first =0;
               if (auto_reset)
                  sent_time=cur_time;
               return true;

           }
        else
           return false;
    };
    int sent_time;
    int sendPeriod;
    uint8_t first=1;
};

class STMsgTag
{
    public:
      STMsgTag(){len=0;};
      uint8_t msg[MAX_TAG_MSG];
      uint8_t len;
      uint8_t priority;
      uint8_t Len(){return len;};
      uint8_t CopyTo(void *p) {memcpy(p,msg,len);return len;};
   
};

class STMsgGPS : public STMsgTag
{   
  public:
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

class STMsgGPSStation : public STMsgTag
{   
  public:
    STMsgGPSStation ( ){len =15;};
    STMsgGPSStation ( float lon, float lat, float z, short dop)
       {
        msg[0]= TAG_GPS_STATION;
        memcpy(&msg[1],&lon,4);
        memcpy(&msg[1+4],&lat,4);
        memcpy(&msg[1+8],&z,4);
        memcpy(&msg[1+12],&dop,2);
        len= 15;
        }
};

class STMsgLORA_HF : public STMsgTag
{
    public:
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
class STMsgSOC : public STMsgTag
{
    public:
    STMsgSOC ( ){len=2;};
    STMsgSOC ( uint8_t soc)
        {
   	    msg[0]=TAG_SOC;
	    msg[1]=soc;
       // printf("Soc set to %d\n",soc);
	    len =2;
        }
};

class STMsgMotion : public STMsgTag
{
    public:
    STMsgMotion ( ){len=2;};
    STMsgMotion ( uint8_t active)
        {
   	    msg[0]=TAG_MOTION;
	    msg[1]=active;
       // printf("Soc set to %d\n",soc);
	    len =2;
        }
};

class STMsgAlert : public STMsgTag
{
    public:
        STMsgAlert(){len=3;};
        STMsgAlert ( uint8_t level, const char *txt)
                {
                    msg[0]=TAG_ALERT;
                    msg[1]=level;
                    msg[2]=strlen(txt);
                    memcpy(&msg[3],txt,msg[2]);
                    len = msg[2]+3;
            	};
        static uint8_t ParseLevel(uint8_t *p){return p[1];};
        static String  ParseText(uint8_t *p){
                                    char buff[20];
                                    memcpy(buff,&p[3],p[2]);
                                    buff[p[2]]=0;
                                    String r((const char *)buff);
                                    return r;
        };
        static uint8_t ParseLen(uint8_t *p){return p[2]+3;};
};

class STMsgPanic : public STMsgTag
{
    public:
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
                };
};

class STMsgSetParam : public STMsgTag
{
    public:
        STMsgSetParam(){len=1;};
        static String GetParamName(uint8_t id){
            switch (id)
              {
                  case PARAM_LINE1:return String("assetName1");
                  case PARAM_LINE2:return String("assetName2");
              }
              return String("");
        }
        static uint8_t ParseId(uint8_t *p){return p[1];};
        static String ParseValue (uint8_t *p){
                                    char buff[20];
                                    memcpy(buff,&p[4],p[2]);
                                    buff[p[2]-4]=0;
                                    String r((const char *)buff);
                                    return r;
                                    };

        static uint8_t ParseSeq (uint8_t *p){
                                    return p[3];
                                    };


        STMsgSetParam ( uint8_t param_id, const char *param_value, uint8_t seq_num)
                {
                msg[0]=TAG_SET_PARAM;
                msg[1]=param_id;
                strcpy((char *)&msg[4],param_value);
                len=4+strlen(param_value);
                msg[2]=len;
                msg[3] = seq_num;
                };
        
};


class STMsgSetResp : public STMsgTag
{
    public:
        STMsgSetResp(){len=1;};
        static String GetParamName(uint8_t id){
            switch (id)
              {
                  case PARAM_LINE1:return String("assetName1");
                  case PARAM_LINE2:return String("assetName2");
              }
              return String("");
        }
        static uint8_t ParseId(uint8_t *p){return p[1];};
        static String ParseValue (uint8_t *p){
                                    char buff[20];
                                    memcpy(buff,&p[4],p[2]);
                                    buff[p[2]-4]=0;
                                    String r((const char *)buff);
                                    return r;
                                    };

        


        STMsgSetResp ( uint8_t param_id, const char *param_value, uint8_t seq_num)
                {
                msg[0]=TAG_SET_RESP;
                msg[1]=param_id;
                strcpy((char *)&msg[4],param_value);
                len=4+strlen(param_value);
                msg[2]=len;
                msg[3] = seq_num;
                };
        
};
#include <iostream> 
#include <list> 
#include <iterator> 
using namespace std; 
/*
void TestMe()
{
    list <STMsgTag> list;

    STMsgPressure p(10);
    list.push_back(p);

}
*/