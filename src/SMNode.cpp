#include "SMNode.h"
#include "stdio.h"
#include "stdlib.h"
#include <stdio.h>
#include <string.h>
SMNode::SMNode()
{
  
}
void SMNode::NodeInit()
{
    int i;
  nCodingRate = 1;
  nSpreadFactor = 7;
  nBandwidth = 125000;
  nFrequencyNdx = FREQ_915;
  bActive = false;
  nConnectionId = -1;
  nTimeslot = -1;
  nGatewayId = -1;
//  nDataTx = 0;
//  nFailCnt = 0;
  connectState=NodeLookForBeacon;
  myMac[0]=1;
  myMac[1]=1;
  myMac[2]=1;
  myMac[3]=4;
  beaconReceived=0;
  stop=0;
  mode = MODE_NORMAL;
  role = ROLE_NORMAL;
  otaMode =0;
//  strcpy(configSSID,"PTG");
//  strcpy(configPWD,"55aa55aa55");
}

void PrintMac(unsigned char *m)
{
extern  char * GetMacString(unsigned char *baseMac);

  printf("%s\n",GetMacString(m));
}
int SMNode::ProcessGrantMsg(unsigned char *p,int len)
{
    GATEWAY_MsgGrant *pMg = (GATEWAY_MsgGrant *)p;
    int i;
    char badMac=0;
    // lets see if its for us
    
  for (i = 0; i < sizeof(pMg->MacId); i++)
      if(pMg->MacId[i] != myMac[i])
         badMac=1;
  // not for us
  if (badMac)
       {
       printf("Bad mac my:");
       PrintMac(myMac);
       printf("Granted ");
       PrintMac(pMg->MacId);
       printf("\n");
       
       return sizeof(GATEWAY_MsgGrant);
       }
   printf ("Good MAC for us\n");
   nSpreadFactor=pMg->nSpreadFactor;
   nTimeslot =  pMg->Timeslot;
  //pMg->TimeslotMask[0] = 0; //TBD
  //pMg->TimeslotMask[1] = 0; //TBD
   nChannel= pMg->nChannel;
   nConnectionId= pMg->ConnectionId;
   nGatewayId= rxGatewayID;
   printf("GRanted SF: %d TS0: %d : Ch: %d CID %d\n",
        nSpreadFactor, nTimeslot, nChannel,nConnectionId);
   connectState = NodeActive;
   configMinHdop = 1;

   return sizeof(GATEWAY_MsgGrant);

    
}

void DumpByteData(const char *cTitle, unsigned char *pData, int nLen);
#include "SMScheduler.h";
extern SMScheduler sched;
int SMNode::ProcessDownMessage(unsigned char *p,int len)
{
    NODE_DownData *downData = (NODE_DownData *)p;
    
    printf("Got Downstream Data me %d for %d len:%d\n",nConnectionId,downData->ConnectionId,downData->payloadLen);
    DumpByteData("DS data:\n",&p[sizeof(NODE_DownData)],downData->payloadLen);
    if (downData->ConnectionId ==nConnectionId)
      {
      memcpy(rxData,p+ sizeof(NODE_DownData),downData->payloadLen);
      rxDataLen=downData->payloadLen;
      }
    sched.RequestScreenRefresh();

    return sizeof(NODE_DownData)+downData->payloadLen;   
    
}
int SMNode::ProcessTimestampMsg(unsigned char *p,int len)
{
      GATEWAY_MsgTimestamp *pMt = (GATEWAY_MsgTimestamp *)p;
     
      if (pMt->nMsgType != BcnTimestampMsg)
          printf("We have a bug\n");
          


      rxEpoch= pMt->nTimestamp;
      rxGatewayID=pMt->nGATEWAYId;
      beaconReceived=1;
      return sizeof(GATEWAY_MsgTimestamp);
}
int SMNode::ProcessRxPack(LORA_PKT *pkt)

{
  unsigned char *p;
  int len;
  unsigned char msgType;
  int plen;

  rxRSSI=pkt->rssi;
  len = pkt->nSize;
  p=pkt->Buf;
  while (len)
      {
       msgType = *p;
      // printf("Got type %d\n",msgType);
       switch (msgType)
          {
            case BcnTimestampMsg: plen=ProcessTimestampMsg(p,len);
                  nRSSI=pkt->rssi;
                  break;
            case BcnGrantMsg: 
                    printf("Got gran\n");
                    plen=ProcessGrantMsg(p,len);
                  break;
            case TypeUnicast:
                    plen = ProcessDownMessage(p,len);
                    break;
            default:
                  printf("Unknonwn Tag %d %d\n",pkt->nSize,len);
                  return (0);
          }
       len-=plen;
       p=p+plen;
      }
      return 1;
}


int SMNode::BuildJoinMessage(LORA_PKT *pkt, int gatewayId, int rssi)

{
  int i;
  NODE_MsgJoin *pJoinMsg;
  pJoinMsg = (NODE_MsgJoin*) pkt->Buf;
  pJoinMsg->nMsgType = NodeJoinMsg;
  for (i=0;i<6;i++)
       pJoinMsg->MacId[i]=myMac[i];
   pJoinMsg->nGATEWAYId=gatewayId;
   pJoinMsg->nRSSI=rssi;
   pkt->nSize = sizeof(NODE_MsgJoin);
   return 1;
}

int SMNode::BuildUpstreamMessage(LORA_PKT *pkt, int nRSSI,unsigned char *payload, int len)


{
  NODE_UpData *p = (NODE_UpData *) pkt->Buf;
  
  p->nMsgType=NodeUpData;
  p->ConnectionId=nConnectionId;
  p->payloadLen=len;
  p->nGATEWAYId=nGatewayId;
  memcpy(&pkt->Buf[sizeof(NODE_UpData)],payload, len);
  pkt->nSize= sizeof(NODE_UpData)+len;
  return pkt->nSize;
//  unsigned char Payload[MAX_NODE_DATA];

}

