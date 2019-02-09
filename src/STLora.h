
#include <sx126x-hal.h>
#include "SMCommon.h"
extern RadioCallbacks_t RadioEvents;

#define POWER_OFF 1
#define POWER_OFF_RETAIN 2
#define POWER_ON 3

class STLora : public  SX126xHal
{
    public:
     STLora (void):SX126xHal(1, 2, 3, 4, 5, 6, NC, NC, NC, NC, NC, NC,&RadioEvents)
    {

    };
    void setup(void);
    uint8_t ReceivePacket(LORA_PKT *pkt);
    uint8_t LoraSend(LORA_PKT *pkt);
    void LoraPrepSend(LORA_PKT *pkt);
    void DoRangeSlave(void);
    uint8_t PacketDetected(void);
    void SetPowerMode(uint8_t md);
    uint8_t mode_rx;
    uint8_t RXEN;
    uint8_t TXEN;
    int  statRXGood;
    int  statRXBad;
    PacketParams_t PacketParams;
    void SetFrequency(int val);
};