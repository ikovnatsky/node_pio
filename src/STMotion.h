
#define STATE_WALKING 1
#define STATE_STILL 0

#include "SparkFunIMU.h"
#include "SparkFunLSM303C.h"
#include "LSM303CTypes.h"

class STMotion 
{
public:
  LSM303C myIMU;
  float acc_vec[3];
  float acc_avg[3] ={0,0,0};
  int counts=50;

  STMotion(){th=5;};
  uint8_t GetActivity();
  void Process(void);
  uint8_t IsActive(void);
  void setup();
  float th;
  float vsum;
  uint8_t state;

};
