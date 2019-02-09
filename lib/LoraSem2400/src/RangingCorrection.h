#ifndef __SX1280_RANGING_CORRECTION_H__
#define __SX1280_RANGING_CORRECTION_H__

#include "sx1280.h"

namespace Sx1280RangingCorrection
{
double GetRangingCorrectionPerSfBwGain( const HFRadioLoRaSpreadingFactors_t sf, const HFRadioLoRaBandwidths_t bw, const int8_t gain);
double ComputeRangingCorrectionPolynome(const HFRadioLoRaSpreadingFactors_t sf, const HFRadioLoRaBandwidths_t bw, const double median);
}

#endif // __SX1280_RANGING_CORRECTION_H__
