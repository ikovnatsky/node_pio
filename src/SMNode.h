
#include "SMCommon.h"
#include "list"

#define MODE_NORMAL 0
#define MODE_DEBUG  1
#define ROLE_NORMAL 0
#define ROLE_BEACON 1
typedef struct
{
  float lon,lat,z;
  uint32_t id;
}HFBeacon;

typedef struct
{
  uint32_t id;
  uint16_t dist;
  uint8_t rssi;
}RANGE_RESULT;

class SMNode
{
  public:
  
  SMNode();
  void NodeInit();
  char connectState;

    int nCodingRate;
    int nSpreadFactor;
    int nBandwidth;
    unsigned char nFrequencyNdx;
    int nRSSI;    // from Beacon Join Message, or other messages from Beacon
    bool bActive;
    static unsigned char MacId[6];
    short nConnectionId;
    short nTimeslot;
    char nChannel;
    unsigned char nGatewayId;

    int FindBeaconSendJoin();
    int ProcessRxPack(LORA_PKT *pkt);
    
    int ProcessTimestampMsg(unsigned char *p,int len);
    int ProcessDownMessage(unsigned char *p,int len);
    int ProcessGrantMsg(unsigned char *p,int len);

    int BuildJoinMessage(LORA_PKT *pkt, int gatewayId, int rssi);
    int BuildUpstreamMessage(LORA_PKT *pkt, int nRSSI, unsigned char *payload, int len);

    int rxEpoch;
    int rxGatewayID;
    int rxRSSI;
    unsigned char otaMode;
    unsigned char myMac[6];

    // configuration settings
    int  configRefresh;
    std::list<HFBeacon> HFBeaconList;  
    uint16_t            hfDelayCal;
    std::list<long> bands;
    uint32_t rangingAddress;
    char configSSID[32];
    char configPWD[32];
    float configMinHdop;
    uint8_t rangeTrys;
    uint8_t rangeReps;
    uint8_t rangeNumBeacons;
    uint8_t ledPWM;
    std::list<RANGE_RESULT> RangeResults;
    
    char line1[30];
    char line2[30];
    
    int8_t zoneOffset;
    // for Now
    unsigned char rxData[30];
    unsigned char rxDataLen;
    unsigned char beaconReceived; 
    unsigned char  stop;
    unsigned char  mode;
    unsigned char  role;
};


enum {
  NodeLookForBeacon,
  NodeWaitJoinState,
  NodeActive
};

