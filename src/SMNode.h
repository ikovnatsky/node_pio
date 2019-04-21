
#include "SMCommon.h"
#include "list"

#include "STMsgTag.h"

#define MODE_NORMAL 0
#define MODE_DEBUG  1
#define ROLE_NORMAL 0
#define ROLE_BEACON 1


#define CHARGE_STATE_NONE 0
#define CHARGE_STATE_CHARGING 1
#define CHARGE_STATE_FULL  2
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
    unsigned short nTimeslot;
    char  nChannel;
    unsigned short nTimeslotMask;
    uint8_t maxTimeSlotLen;
    unsigned char nGatewayId;

    int FindBeaconSendJoin();
    int ProcessRxPack(LORA_PKT *pkt);
    
    int ProcessTimestampMsg(unsigned char *p,int len);
    int ProcessDownMessage(unsigned char *p,int len);
    int ProcessGrantMsg(unsigned char *p,int len);
    int ParseDownMessage(uint8_t *p, int len);
    int BuildJoinMessage(LORA_PKT *pkt, int gatewayId, int rssi);
    int BuildUpstreamMessage(LORA_PKT *pkt, int nRSSI, unsigned char *payload, int len);
    int BuildUpstreamMessage(LORA_PKT *pkt, int nRSSI, int max_len);

    uint8_t rxEpoch;
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
    
    std::list<STMsgTag> upstreamQ;

    char line1[30];
    char line2[30];
    
    int8_t zoneOffset;
    // for Now
    char alertText[30];
    unsigned char alertLevel;
    unsigned char rxDataLen;
    unsigned char beaconReceived; 
    unsigned char  stop;
    unsigned char  mode;
    unsigned char  role;
    unsigned char  panic;
    unsigned char configChanged;
    uint8_t soc;  // state of charge
    uint8_t charge_state;
    float    avgI;
    float press;
    float ttE;

    int loopCnt;
    int opModeSpot;
    int last_send_time;

    // Send timers
    TagSendTimer socTimer,motTimer,pressTimer;

    String fwServer;
    String fwImage;
};


enum {
  NodeLookForBeacon,
  NodeWaitJoinState,
  NodeActive
};

