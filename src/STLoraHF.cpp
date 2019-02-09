#include "Arduino.h"
#include "STLoraHF.h"
#include "sx1280-hal.h"

HFRadioCallbacks_t HFRadioEvents =

    {
        NULL, // txDone
        NULL, // rxDone
        NULL, // rxPreambleDetect
        NULL, // rxSyncWordDone
        NULL, // rxHeaderDone
        NULL, // txTimeout
        NULL, // rxTimeout
        NULL, // rxError
        NULL, // cadDone
};

SX1280Hal LoraRadHF(0, 0, 0, 0, 0, 0, 0, 0, 0, &HFRadioEvents); // = new LoRaM(SS, 0, 0);

HFModulationParams_t ModulationParams;
HFPacketParams_t PacketParams;
    double RngFeiFactor;
    double RngFei;

const uint16_t RNG_CALIB_1600[] = {13308, 13493, 13528, 13515, 13430, 13376};
//const double   RNG_FGRAD_0400[] = { -0.148, -0.214, -0.419, -0.853, -1.686, -3.423 };
//const double   RNG_FGRAD_0800[] = { -0.041, -0.811, -0.218, -0.429, -0.853, -1.737 };
const double RNG_FGRAD_1600[] = {0.103, -0.041, -0.101, -0.211, -0.424, -0.87};

void STLoraHF::LSTAT()
{
    HFRadioStatus_t stat;
    stat = LoraRadHF.GetStatus();

    printf(" %d %d Stat:%d mode:%d\n",
           stat.Fields.CpuBusy,
           stat.Fields.DmaBusy,
           stat.Fields.CmdStatus,
           stat.Fields.ChipMode);
}

void STLoraHF ::SetRangingCalibration(short val)
{
    LoraRadHF.SetRangingCalibration(val);
}
void STLoraHF::SetRangingAddress(uint32_t add)
{
  printf("Setting ranging address %x\n",add);
   LoraRadHF.SetDeviceRangingAddress(add);

}
void STLoraHF::setup()
{
    pinMode(SS_LORAHF, OUTPUT);

#define RF_FREQUENCY 2402000000UL // Hz

    LoraRadHF.gRadioCS = SS_LORAHF;
    LoraRadHF.RadioSpi = &SPI;
    LoraRadHF.Init();
    LoraRadHF.SetRegulatorMode(USE_LDO); // Can also be set in LDO mode but consume more power
    LoraRadHF.SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);

    //   SetDeviceRangingAddress( RngAddress );
    //   SetRangingCalibration( RngCalib );

    ModulationParams.PacketType = PACKET_TYPE_LORA;

    PacketParams.PacketType = PACKET_TYPE_LORA;

    /// ranging config
    uint16_t RngCalib;
    uint16_t RngReqDelay;
    uint32_t RngAddress = 0x32100000; //0x10000000;

    ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF8;
    ModulationParams.Params.LoRa.Bandwidth = LORA_BW_1600;
    ModulationParams.Params.LoRa.CodingRate = LORA_CR_4_5;

    PacketParams.Params.LoRa.PreambleLength = 0x08;
    PacketParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
    PacketParams.Params.LoRa.PayloadLength = 9;
    PacketParams.Params.LoRa.Crc = LORA_CRC_ON;
    PacketParams.Params.LoRa.InvertIQ = LORA_IQ_INVERTED;

    RngCalib = RNG_CALIB_1600[(ModulationParams.Params.LoRa.SpreadingFactor >> 4) - 5];
    RngCalib = RngCalib;
    RngFeiFactor = (double)RNG_FGRAD_1600[(ModulationParams.Params.LoRa.SpreadingFactor >> 4) - 5];
    //  RngReqDelay  = RNG_TIMER_MS >> ( 2 + 10 - ( ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );

    // this devices address
    LoraRadHF.SetDeviceRangingAddress(RngAddress);
    LoraRadHF.SetRangingIdLength(RANGING_IDCHECK_LENGTH_32_BITS);

    //SetRangingRequestAddress
   // LoraRadHF.SetRangingCalibration(RngCalib);
    ModulationParams.PacketType = PACKET_TYPE_RANGING;
    PacketParams.PacketType = PACKET_TYPE_RANGING;

    LoraRadHF.SetStandby(STDBY_RC);
    LoraRadHF.SetPacketType(ModulationParams.PacketType);
    LoraRadHF.SetModulationParams(&ModulationParams);
    LoraRadHF.SetPacketParams(&PacketParams);

    LoraRadHF.SetRangingCalibration(RngCalib);
    LoraRadHF.SetDeviceRangingAddress(RngAddress);

    LoraRadHF.SetRfFrequency(RF_FREQUENCY);
    LoraRadHF.SetBufferBaseAddresses(0x00, 0x00);
    LoraRadHF.SetTxParams(2, RADIO_RAMP_20_US);
    LoraRadHF.SetDioIrqParams(IRQ_RANGING_MASTER_RESULT_VALID, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
    uint8_t sw[] = {0xDD, 0xA0, 0x96, 0x69, 0xDD};
    LoraRadHF.SetSyncWord(1, sw);
    printf("Sync word  ");
    LSTAT();

    LoraRadHF.SetFs();
    printf("FS    ");
    LSTAT();

    LoraRadHF.SetStandby(STDBY_RC);
    printf("standby    ");
    LSTAT();

    //while (1)
    for (int x = 0; x < 2; x++)
    {
        printf("FW version %d\n", LoraRadHF.GetFirmwareVersion());
        delay(10);
    }
}
#include "RangingCorrection.h"
#define RANGE_TO 60

uint8_t STLoraHF:: SingleRangeToSlave (uint32_t id, uint8_t to, float &dist, int8_t &rssi)
{
    LoraRadHF.SetRangingRequestAddress(id);
    LoraRadHF.ClearIrqStatus(IRQ_RANGING_MASTER_RESULT_VALID);
    LoraRadHF.SetTxParams(0, RADIO_RAMP_20_US);
HFPacketStatus_t packetStatus ;

  //  LoraRadHF.SetTxContinuousWave();
   // while(1)
     // delay(100);
    //return 0;
    LoraRadHF.SetTx((HFTickTime_t){RADIO_TICK_SIZE_1000_US, 0xFFFF});
   // vTaskDelay(10);

    while (((LoraRadHF.GetIrqStatus() & IRQ_RANGING_MASTER_RESULT_VALID)==0) && --to)
        vTaskDelay(1);
    if (to>0)
    {
        dist = LoraRadHF.GetRangingResult(RANGING_RESULT_RAW);
        rssi=  LoraRadHF.GetRssiInst();

        //int snr = LoraRadHF.Get
        LoraRadHF.GetPacketStatus(&packetStatus);
        printf("Ranging good to 0x%x to %d %d %f  snr %d %d\n",id,to,rssi,dist,packetStatus.LoRa.SnrPkt,packetStatus.LoRa.RssiPkt);
        return 1;
    }
    //printf("Ranging failed %x  %x\n",id,LoraRadHF.GetIrqStatus());
    return 0;
}

int8_t STLoraHF::  GetRssi(void)
{
    HFPacketStatus_t packetStatus ;
    LoraRadHF.GetPacketStatus(&packetStatus);
    return packetStatus.LoRa.RssiPkt;
}

uint8_t STLoraHF:: RangeToBeacon(uint32_t id, int8_t reps, int8_t trys, float &dist)
{
    float dist_sum=0;
    float d;
    int8_t rssi;
    int count=0;
    //printf("Try range to %x\n",id);
    for (int i = 0;i<reps;i++)
       {
           if (SingleRangeToSlave(id,RANGE_TO,d,rssi))
            {
              //printf("Ranged to %x %f %d\n",id,d,count);
              dist_sum+=d;
              count++;
            }
        else
            trys--;
        if (trys==0)
           break;
       }

    if (count>0)
          {
              dist = dist_sum/count;
              return count;
          }
       
    return count;
}

void STLoraHF::SetSleep(uint8_t en)
{
    if (en)
      {
          uint8_t sleep = 0;
      LoraRadHF.SetStandby(STDBY_RC);
          LoraRadHF.WriteCommand( RADIO_SET_SLEEP, &sleep, 1 );

      }
}
void STLoraHF ::RangeToSlave(uint32_t id)
{
    //  printf("irq = %x \n",GetIrqStatus());
    double res[1];
    LoraRadHF.SetTx((HFTickTime_t){RADIO_TICK_SIZE_1000_US, 0xFFFF});
    delay(300);
    //SendPayload( Buffer, 5,  (HFTickTime_t ) { RADIO_TICK_SIZE_1000_US, 0xFFFF } );
    //  printf("Sent \n");
    // printf("irq = %x \n",GetIrqStatus());
    //  stat=GetStatus();
    RngFei=3100;
    if (LoraRadHF.GetIrqStatus() & IRQ_RANGING_MASTER_RESULT_VALID)
    {
        res[0] = LoraRadHF.GetRangingResult(RANGING_RESULT_RAW);
        hf_locs[0].dist=(short)res[0];
        hf_loc_count=1;
        printf("Ranged is %3.3f  %d\n", (float)res[0],
                                        LoraRadHF.GetRssiInst());
    }
    else
    hf_locs[0].dist=10000;
    printf ("Method 2 %3.3f\n", (float) Sx1280RangingCorrection::ComputeRangingCorrectionPolynome(LORA_SF8, LORA_BW_1600,res[0]));
    CalcDistance( res,1,  RngFeiFactor,  RngFei,   LoraRadHF.GetRssiInst() );

    /*printf("%x %x\n  %d %d stat:%d mode%d\n",len, Buffer[0],
     stat.Fields.CpuBusy,
     stat.Fields.DmaBusy,
     stat.Fields.CmdStatus, 
     stat.Fields.ChipMode);
*/

}


       //GetPacketStatus( &pktStatus );
        //pkt->SNR = pktStatus.Params.LoRa.SnrPkt;
double STLoraHF ::GetFrequencyError()
{
    return LoraRadHF.GetFrequencyError();
}
void STLoraHF ::DoRangeSlave(void)
{
    HFRadioStatus_t stat;
    uint8_t
     len;
    uint8_t Buffer[10];
     LoraRadHF.ClearIrqStatus(0x180);
     LoraRadHF.SetDioIrqParams( 0x180,0,0,0);
    LoraRadHF.SetRx((HFTickTime_t){RADIO_TICK_SIZE_1000_US, 0xffff});
    printf("Setup for auto return \n");
    return;
    while (1)
    {
        //    LSTAT();

        //   LoraRadHF.GetPayload((uint8_t *) Buffer, &len, 10 );
        stat = LoraRadHF.GetStatus();
        printf("irq = %x %x\n", LoraRadHF.GetIrqStatus(), stat.Value);
        len = 0;
   //     if (((uint8_t)stat&5)==5)
        if (LoraRadHF.GetIrqStatus()&0x180)
        {
            LoraRadHF.GetPayload((uint8_t *)Buffer, &len, 10);
            printf("len =%x %x\n  %d %d Stat:%d mode%d  irq %x   ", len, Buffer[0],
                   stat.Fields.CpuBusy,
                   stat.Fields.DmaBusy,
                   stat.Fields.CmdStatus,
                   stat.Fields.ChipMode,
                   LoraRadHF.GetIrqStatus());
            printf("error = %f\n",(float)LoraRadHF.GetFrequencyError( ));
            LoraRadHF.ClearIrqStatus(0x180);
            //return;
        }
   // vTaskDelay(100);
    }

}


RANGE_RES_TYPE t0 =       -0.016432807883697;                         // X0
RANGE_RES_TYPE t1 =       0.323147003165358;                          // X1
RANGE_RES_TYPE t2 =       0.014922061351196;                          // X1^2
RANGE_RES_TYPE t3 =       0.000137832006285;                          // X1^3
RANGE_RES_TYPE t4 =       0.536873856625399;                          // X2
RANGE_RES_TYPE t5 =       0.040890089178579;                          // X2^2
RANGE_RES_TYPE t6 =       -0.001074801048732;                         // X2^3
RANGE_RES_TYPE t7 =       0.000009240142234;                          // X2^4

char STLoraHF ::CalcDistance( RANGE_RES_TYPE *RawRngResults, int RngResultIndex, RANGE_RES_TYPE RngFeiFactor, RANGE_RES_TYPE RngFei, RANGE_RES_TYPE RssiValue )
{
    uint16_t j = 0;
    uint16_t i;
    RANGE_RES_TYPE rssi = RssiValue;
    RANGE_RES_TYPE median;
    
    if( RngResultIndex > 0 ){
        for( i = 0; i < RngResultIndex; ++i ){
            RawRngResults[i] = RawRngResults[i] - ( RngFeiFactor * RngFei / 1000 );
            printf("\r\nRawFei: %f ",RawRngResults[i]);
        }

        for (int i = RngResultIndex - 1; i > 0; --i){
            for (int j = 0; j < i; ++j){
                if (RawRngResults[j] > RawRngResults[j+1]){
                    int temp = RawRngResults[j];
                    RawRngResults[j] = RawRngResults[j+1];
                    RawRngResults[j+1] = temp;
                }
            }
        }
        
        if ((RngResultIndex % 2) == 0){
            median = (RawRngResults[RngResultIndex/2] + RawRngResults[(RngResultIndex/2) - 1])/2.0;
        } 
        else{
            median = RawRngResults[RngResultIndex/2];
        }
        
        if( median < 50 ){
            median = t0 + t1 * rssi + t2 * pow(rssi,2) + t3 * pow(rssi, 3) +t4 * median + t5 * pow(median,2) + t6 * pow(median, 3) + t7 * pow(median, 4) ;
            for( int i = 0; i < RngResultIndex; ++i ){
            // Apply the short range correction and RSSI short range improvement below 50 m
            RawRngResults[i] = t0 + t1 * rssi + t2 * pow(rssi,2) + t3 * pow(rssi, 3) +t4 * RawRngResults[i] + t5 * pow(RawRngResults[i],2) + t6 * pow(RawRngResults[i], 3) + t7 * pow(RawRngResults[i], 4) ;    
            printf( "\r\n%5.3f", RawRngResults[i] );
            }
        }
        

    }
    /*for( int i = 0; i < RngResultIndex; ++i ){
        printf("%5.3f\r\n", RawRngResults[i]);
    }*/
    median=0;
    printf( ": Rssi: %4.0f, Median: %5.1f, FEI: %f\r\n",  rssi, median, RngFei );
    return 0;
}  