
#ifndef SMCOMMON
#define SMCOMMON
const long BandFrequencyTable[] = { 915000000, 914000000, 916000000}; // determined bny FCC, use to expand number of gateways that can control more nodes
#define MAX_FREQ_BANDS (sizeof(BandFrequencyTable)/sizeof(BandFrequencyTable[0]))
enum {FREQ_915, FREQ_914, FREQ_916};    // Channels
// Node state machine on GATEWAY
enum {NodeFreeState, NodeJoinedState};

// LoRa Radio message types,PTG defined
// (GATEWAYMsgTypes;)
enum  {
  BcnTimestampMsg = 2,
  BcnGrantMsg,
  NodeJoinMsg,
  NodeUpData,
  NodeAckData,
  TypeAck,
  TypeUnicast,
  TypeBroadcast,
  TypeDone,
  TypeKeepListening,
  MaxGATEWAYMsgTypes
  };
/*
    Node defines here
*/
#define MAX_NODES 32
#define MAX_NODE_DATA 32


typedef struct {
  unsigned char nMsgType;
  unsigned char nGATEWAYId;
  unsigned char nTimestamp;   // MSB
} GATEWAY_MsgTimestamp;

typedef struct {
  unsigned char nMsgType;
//  unsigned char nGATEWAYId;
//  unsigned char nTimestamp;   // MSB
//  unsigned char nExtendType;
  unsigned char MacId[6];
  unsigned short ConnectionId;        // ConnectionId
  unsigned char nSpreadFactor;
  unsigned short Timeslot;      // Timeslot
  unsigned char TimeslotMask[2];  // Timeslot Mask ??? TBD
  unsigned char nChannel;
} GATEWAY_MsgGrant;


typedef struct {
  unsigned char nMsgType;
  unsigned char MacId[6];
  unsigned char nGATEWAYId;
  signed char nRSSI;
} NODE_MsgJoin;

typedef struct {
  unsigned char nMsgType;
  unsigned char MacId[6];
  unsigned char nGATEWAYId;
  unsigned char ConnectionId[2];        // ConnectionId
} NODE_AckData;

typedef struct {
  unsigned char nMsgType;
  unsigned short ConnectionId;
  unsigned char nGATEWAYId;
  unsigned char payloadLen;
 // unsigned char Payload[MAX_NODE_DATA];
} NODE_UpData;

typedef struct {
  unsigned char nMsgType;
  unsigned short ConnectionId;
  unsigned char payloadLen;
 // unsigned char Payload[MAX_NODE_DATA];
} NODE_DownData;

#define MAX_LORA_PKT 80
typedef struct {
  int nSize;
  unsigned char Buf[MAX_LORA_PKT];
  float SNR;
  int rssi;
} LORA_PKT;

typedef struct {
  unsigned char nMsgType;
  unsigned char MacId[6];
  unsigned char nGATEWAYId;
  unsigned char ConnectionId[2];        // ConnectionId
} NODE_HDR;
#endif