
#define SS_LORAHF 14

#define RANGE_ADDRESS_BASE 0x32100000
//#include "sx1280-hal.h"
#define RANGE_RES_TYPE double
#define BEACONID short
typedef struct
{
  BEACONID id;
  short dist;
}HFLoc;

class STLoraHF 
{
  public:
    STLoraHF(){hf_loc_count=0;};
    void setup();
    void LSTAT();
    void RangeToSlave(uint32_t id);
    void DoRangeSlave(void);
    void SetSleep(uint8_t en);
    double GetFrequencyError();
    void SetRangingCalibration(short val);
    char CalcDistance( RANGE_RES_TYPE *RawRngResults, int RngResultIndex, RANGE_RES_TYPE RngFeiFactor, RANGE_RES_TYPE RngFei, RANGE_RES_TYPE RssiValue );
    HFLoc hf_locs[4];
    unsigned char hf_loc_count;

    uint8_t SingleRangeToSlave (uint32_t id, uint8_t to, float &dist, int8_t &rssi);
    uint8_t RangeToBeacon(uint32_t id, int8_t reps, int8_t trys, float &dist);
    void SetRangingAddress(uint32_t add);
    int8_t GetRssi(void);

};
