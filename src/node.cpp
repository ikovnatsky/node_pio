#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "Arduino.h"
#include <SPI.h>
#include "SPIFFS.h"

//#include <LoRa.h>
#include <Wire.h>
//#include "SSD1306.h"
#include "esp32-hal-timer.h"
#include "freertos/xtensa_api.h"
#include "rom/ets_sys.h"
#include "soc/timer_group_struct.h"
#include "soc/dport_reg.h"
#include "esp_attr.h"
#include "esp_intr.h"

#include "esp_system.h"
#include "SMScheduler.h"
#include "SMNode.h"

#include "smessage.h"
#include "STHardware.h"
#include "STLora.h"
#include "STLoraHF.h"
#include "STGps.h"
#include "STio.h"
#include "STDisplay.h"
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include "STLed.h"
#include <Thanos_MAX17055.h>
#include "max17055.h"
#include "qrcode.h"
#include "STButtons.h"


#include "SparkFunIMU.h"
#include "SparkFunLSM303C.h"
#include "LSM303CTypes.h"
#include "STMotion.h"
#include <LPS25HBSensor.h>


const unsigned char bitmapAntenna [] PROGMEM=
{
0x00, 0x00, 0x00, 0x7f, 0xff, 0x80, 0x60, 0xc1, 0x80, 0x30, 0xc3, 0x00, 0x18, 0xc6, 0x00, 0x0c,
0xcc, 0x00, 0x06, 0xd8, 0x00, 0x06, 0xd8, 0x00, 0x03, 0xf0, 0x00, 0x01, 0xe0, 0x00, 0x00, 0xc0,
0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00, 0x00, 0xc0, 0x00,
0x00, 0xc0, 0x00, 0x00, 0x00, 0x00
};
// 'charge', 26x47px
const unsigned char chargeBitmap[] PROGMEM = {
    0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xc0,
    0xff, 0xf8, 0x0f, 0xc0, 0xff, 0xf0, 0x0f, 0xc0, 0xff, 0xf0, 0x0f, 0xc0, 0xff, 0xf0, 0x1f, 0xc0,
    0xff, 0xe0, 0x1f, 0xc0, 0xff, 0xe0, 0x1f, 0xc0, 0xff, 0xe0, 0x3f, 0xc0, 0xff, 0xc0, 0x3f, 0xc0,
    0xff, 0xc0, 0x3f, 0xc0, 0xff, 0xc0, 0x7f, 0xc0, 0xff, 0x80, 0x7f, 0xc0, 0xff, 0x80, 0x7f, 0xc0,
    0xff, 0x00, 0xff, 0xc0, 0xff, 0x00, 0x01, 0xc0, 0xff, 0x00, 0x01, 0xc0, 0xfe, 0x00, 0x03, 0xc0,
    0xfe, 0x00, 0x03, 0xc0, 0xfe, 0x00, 0x03, 0xc0, 0xfc, 0x00, 0x07, 0xc0, 0xff, 0xf8, 0x07, 0xc0,
    0xff, 0xf0, 0x07, 0xc0, 0xff, 0xf0, 0x0f, 0xc0, 0xff, 0xf0, 0x0f, 0xc0, 0xff, 0xe0, 0x0f, 0xc0,
    0xff, 0xe0, 0x1f, 0xc0, 0xff, 0xe0, 0x1f, 0xc0, 0xff, 0xc0, 0x1f, 0xc0, 0xff, 0xc0, 0x3f, 0xc0,
    0xf8, 0x00, 0x01, 0xc0, 0xf8, 0x00, 0x03, 0xc0, 0xfc, 0x00, 0x07, 0xc0, 0xfe, 0x00, 0x0f, 0xc0,
    0xff, 0x00, 0x3f, 0xc0, 0xff, 0x80, 0x7f, 0xc0, 0xff, 0x80, 0xff, 0xc0, 0xff, 0xc1, 0xff, 0xc0,
    0xff, 0xe3, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xc0,
    0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xff, 0xc0};

#define CORE_1 1

enum
{
  DisplayPriority = 1,
  GPSPriority,
  ButtonPriority,
  LoRaSendPriority,
  LoRaNodePriority,
  MonitorPriority,
};
enum
{
  DisplayHandle,
  CommTaskNdx,
  GPSHandle,
  ButtonHandle,
  LoRaSendTaskNdx,
  MonitorTaskNdx,
  MaxTasks
};

TaskHandle_t TaskHandles[MaxTasks];

// module we will be using
SMScheduler sched;
STLora Lora;
STLoraHF LoraHF;
STGps gps;
STDisplay disp;
STLed LedSide(0x61);
STLed LedTop(0x60);
//Thanos_MAX17055 charge_mon;
MAX17055 charge_mon;
STButtons buttons;
STMotion motion;

LPS25HBSensor *altimeter;


void GetMac(unsigned char *mac)
{

   uint64_t m;
  
  // Get MAC address for WiFi station
  m = ESP.getEfuseMac();
  mac[0] = (uint8_t)(m >> 0);
  mac[1] = (uint8_t)(m >> 8);
  mac[2] = (uint8_t)(m >> 16);
  mac[3] = (uint8_t)(m >> 24);
  mac[4] = (uint8_t)(m >> 32);
  mac[5] =  (uint8_t)(m >> 40);
}

char *GetMacString(unsigned char *baseMac)
{
  static char baseMacChr[20];
  sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return baseMacChr;
}

char statusString[20];
void SetStatusString(char *s)
{
  memset(statusString, 0, sizeof(statusString));
  strcpy(statusString, s);
}
/************************************************************************
  DumpByteData -
  utility to dumple data to screen/port.
  Input(s)
  char *Title
  unsigned char *pData
  int nLen
  Output (none)
************************************************************************/
void DumpByteData(const char *cTitle, unsigned char *pData, int nLen)
{
  int i;
  char cOut[128], cTmp[32];

  Serial.printf("%s", cTitle);
  for (i = 0; i < nLen; i++)
  {
    if (!(i & 31))
    {
      sprintf(cTmp, "%02x: ", i);
      Serial.printf("%s", cTmp);
    }
    else
      cTmp[0] = '\0';
    sprintf(cOut, "%02x ", pData[i]);
    if (!((i + 1) & 31))
      strcat(cOut, "\n");
    Serial.printf("%s", cOut);
  }
  Serial.printf("\n");
}

SMNode theNode;



int get_toa(int n_size, int n_sf, int n_bw, int enable_eh, int enable_crc, int n_cr, int n_preamble)
{
    float r_sym = (n_bw*1000.) / pow(2,n_sf);
    float t_sym = 1000. / r_sym;
    float t_preamble = (n_preamble + 4.25) * t_sym;
    int v_CRC = 1;
    int v_IH = 0;
    int v_DE =0;
    /* ldre
    if enable_auto_ldro:
        if t_sym > 16:
            v_DE = 1
    elif enable_ldro:
        v_DE = 1
    # IH
    */
   #define max(a,b) (a>b?a:b)
    if (enable_eh==0)
        v_IH = 1;
    
    if (enable_crc==0)
        v_CRC = 0;
    float a = 8.*n_size - 4.*n_sf + 28 + 16*v_CRC - 20.*v_IH;
    float b = 4.*(n_sf-2.*v_DE);
    float v_ceil = a/b;
    float n_payload = 8 + max(ceil(a/b)*(n_cr+4), 0);
    float t_payload = n_payload * t_sym;
    float t_packet = t_preamble+ t_payload;
    return t_packet + .5;
}
int CalcPacketTime(int len)
{
 // return 0;
 int v;
 v= get_toa(len,7,125,1,1,1,8);
 //printf("\nCalc len %d t %d\n",len,v);
 return v;
  //return 21 + (len * 16) / 10;
}
#define GPIO_DEEP_SLEEP_DURATION     8  // sleep 2 seconds and then wake up
RTC_DATA_ATTR static time_t last;        // remember last boot in RTC Memory
RTC_DATA_ATTR static uint32_t bootcount; // remember number of boots in RTC Memory
#include "sys/time.h"
struct timeval now;

#include "driver/driver/rtc_io.h"
#include <esp_wifi.h>
#include <esp_deep_sleep.h>

void LPTest2()
{
int x;
  gettimeofday(&now, NULL);

  printf("start ESP32 %d\n",bootcount++);
/*
  printf("deep sleep (%lds since last reset, %lds since last boot)\n",now.tv_sec,now.tv_sec-last);
    esp_sleep_enable_timer_wakeup(1000000LL * GPIO_DEEP_SLEEP_DURATION);
  Serial.printf("in light sleep\n");
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM,ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM,ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX,ESP_PD_OPTION_OFF);
  esp_wifi_stop();
  //esp_bt_controller_disable();
  //esp_bluedroid_disable();

  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM,ESP_PD_OPTION_ON);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM,ESP_PD_OPTION_ON);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_MAX,ESP_PD_OPTION_OFF);

    //ESP.deepSleep(1000000LL * GPIO_DEEP_SLEEP_DURATION);
*/
/*
esp_sleep_enable_ext0_wakeup(GPIO_NUM_32,1);
rtc_gpio_hold_en(GPIO_NUM_12);
rtc_gpio_hold_en(GPIO_NUM_1);
rtc_gpio_hold_en(GPIO_NUM_16);
//  esp_deep_sleep_pd_config(ESP_PD_DOMAIN_XTAL,ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,ESP_PD_OPTION_ON);

esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_ON);
//  esp_deep_sleep_start();
  // esp_deep_sleep(1000000LL * GPIO_DEEP_SLEEP_DURATION);
  */
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_XTAL,ESP_PD_OPTION_OFF);
  rtc_gpio_hold_en(GPIO_NUM_12);
  rtc_gpio_hold_en(GPIO_NUM_32);
  rtc_gpio_hold_en(GPIO_NUM_33);
    esp_deep_sleep_start();//32 33
rtc_gpio_hold_en(GPIO_NUM_16);
    esp_light_sleep_start();
    ESP.restart();
}

void LPTest()
{

  gettimeofday(&now, NULL);

  printf("start ESP32 %d\n",bootcount++);

  printf("deep sleep (%lds since last reset, %lds since last boot)\n",now.tv_sec,now.tv_sec-last);


  //  printf("bat infor I %3.3f avgI %3.3f SOC %3.3f vc %3.3f rss %d \n",charge_mon.getCurrent_mA(),charge_mon.getAvgCurrent_mA(),charge_mon.getRepSOC(),charge_mon.getVoltageCell(),LoraHF.GetRssi());
  last = now.tv_sec;

    printf("Lora set to bed\n");
    LoraHF.SetSleep(1);
    delay(4000);
//    Lora.SetSleep(1);

    gps.Enable(0);
  printf("enter deep sleep\n");
  for (int x=1;x<200;x++)
     {
       gps.Process();
       delay(2);
     }
    printf("Lora set to bed\n");
    Lora.SetPowerMode(POWER_OFF_RETAIN);
  //  delay(20);
    printf("Sleeping \n");
/*
   rtc_gpio_hold_en((gpio_num_t)LORA_TXEN);
   rtc_gpio_hold_en((gpio_num_t)LORA_RXEN);
    rtc_gpio_hold_en((gpio_num_t)LORA_RXEN);
    
    gpio_pullup_en((gpio_num_t)MISO);
    rtc_gpio_hold_en((gpio_num_t)MISO);

    rtc_gpio_hold_en((gpio_num_t)SCK);

    gpio_pulldown_en((gpio_num_t)MOSI);
    rtc_gpio_hold_en((gpio_num_t)MOSI);

*/
    /*gpio_pullup_en((gpio_num_t)GPS_TX_PIN);
    gpio_pullup_en((gpio_num_t)CS_3);
    rtc_gpio_hold_en((gpio_num_t)GPS_TX_PIN);
   rtc_gpio_hold_en((gpio_num_t)GPS_RX_PIN);
    rtc_gpio_force_hold_dis_all();
*/

   esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,ESP_PD_OPTION_OFF);
  // esp_deep_sleep(1000000LL * GPIO_DEEP_SLEEP_DURATION);
  
    esp_sleep_enable_timer_wakeup(1000000LL * GPIO_DEEP_SLEEP_DURATION);
  Serial.printf("in light sleep\n");
  delay(10);
    esp_light_sleep_start();
 // DoReset();
  Serial.printf("in deep sleep\n");
  delay(10);
  //IOwriteRegister(TCA6408A_OUTPUT,0);

    //rtc_gpio_pullup_en();
    ESP.restart();



}
void DOHFRange()
{
  // after sending out data we can perform a range using lorahf
    theNode.RangeResults.clear();
    sched.spi_take(200,0);
    float d;
    int cn;
    
for (auto const& i : theNode.HFBeaconList) {
    
    cn=LoraHF.RangeToBeacon(i.id+RANGE_ADDRESS_BASE, theNode.rangeReps,theNode.rangeTrys, d);
    if (cn)
       {
         RANGE_RESULT rs;
         rs.id = i.id;
         rs.dist=d+.5;
         theNode.RangeResults.push_front(rs);
       }
    if (theNode.RangeResults.size()==theNode.rangeNumBeacons)
       break;
    printf("Range is %3.3f %d %x\n",d,cn,i.id);
    // sched.DoUI();
    }
     LoraHF.SetSleep(1);
    //LoraHF.RangeToSlave(0);
    sched.spi_give();

}

short limit_ushort(int val)
{
  if (val<0)
     return 0;
  if (val>65000)
    return 65000;
   return val;
}

#define FREE_FOR_ALL_TS 8
void OpMode()
{
  LORA_PKT LoraPkt;
  int nRSSI;
  int i;
  int missed_beacons = 0;
  int sleep_start;
  int disp_div=0;
  printf("Started active state  task \n");
  
  sched.RequestScreenRefresh();
  // time before timeslot to start setting things up
  #define SETUP_OFFSET -10
  while (theNode.stop == 0 )
  {
    // schedule for the beacon
    // we could do this at the end to save an epoch

    sched.PrintTime("Wait B:-----------------------------------------------------------");
    // we can sleep until its time for a beacon
    Lora.SetPowerMode(POWER_OFF_RETAIN);
    if (theNode.role!=ROLE_BEACON)
            LoraHF.SetSleep(1);
    sched.WaitUntil(0, 0, SETUP_OFFSET, TaskHandles[LoRaSendTaskNdx]);
    Lora.SetPowerMode(POWER_ON);
    //printf("Slepped for %d\n",clock()-sleep_start);
    sched.PrintTime("Start Beacon :");
    
    
    //listen during beacon time
    sched.SetEventTime(FREE_FOR_ALL_TS+10, 0, 0);
    theNode.rxGatewayID = -1;
    while (!sched.EventExpired())
    {
      if (Lora.ReceivePacket(&LoraPkt) == 0)
        {
        continue;
        }
      theNode.ProcessRxPack(&LoraPkt);
      sched.PrintTime("Got Rx:");
      //DumpByteData("Rx:", LoraPkt.Buf, LoraPkt.nSize);
      // must test iTs thE riGhT onE
      if (theNode.rxGatewayID>0)
          {
            sched.ResetTime(CalcPacketTime(LoraPkt.nSize),theNode.rxEpoch);
            break;
          }
    }

    //printf("Gateway %d missed %d \n",theNode.rxGatewayID,missed_beacons);
    if (theNode.rxGatewayID < 0)
      missed_beacons++;
    else
      missed_beacons = 0;

    if (missed_beacons > 3)
    {
      printf("No more beacons must find another--------------- gw\n");
      theNode.connectState = NodeLookForBeacon;
      theNode.nTimeslot = -1;
      theNode.nConnectionId = -1;
      theNode.rxGatewayID = 0;
      sched.RequestScreenRefresh();
      return;
    }
    
    // lets make sure its our epoch if not go back to looking
    if (theNode.nTimeslot>>8!=theNode.rxEpoch)
    // wait a bit before we need to send
    //printf("Scheduling upstream at %d\n", theNode.nTimeslot & 0xff);
    sched.PrintTime("Done beacon");
  #define SEND_PREP_TIME -6
    sched.WaitUntil(theNode.nTimeslot & 0xff, 0, SEND_PREP_TIME);
    sched.PrintTime("Prep");


/// this is where we build the packet to be sent upstream
    {
      unsigned char msg[64];
      int len=0;
      #define BAT_TRANSMIT_DIV 10
      #define P_TRANSMIT_DIV 1
      static uint8_t bat_update_div=BAT_TRANSMIT_DIV;
      static uint8_t p_update_div = P_TRANSMIT_DIV;

      if (gps.location.isValid())
        len += smessage_add_GPS(msg, gps.location.lng(), gps.location.lat(), gps.altitude.meters(), limit_ushort(gps.hdop.value()));
      
      for (auto ranges : theNode.RangeResults)
        len+= smessage_add_LORA_HF(&msg[len],ranges.id,ranges.dist);

      if (theNode.panic)
        len+= smessage_add_PANIC(&msg[len]);

      if(1)
      if (p_update_div--==0)
        {
        float press;
        sched.wire_take(10);
        altimeter->GetPressure(&press);
        altimeter->GetPressure(&press);
        sched.wire_give();
        printf("\nPress = %3.3f\n",press);
        len+= smessage_add_PRESSURE_HPA(&msg[len],press);
        p_update_div=P_TRANSMIT_DIV;
        }
      if (bat_update_div--==0)
           {
             len+=smessage_add_bat (&msg[len], charge_mon.getRepSOC());
             bat_update_div=BAT_TRANSMIT_DIV;
           }
        
      // add rssi
      theNode.BuildUpstreamMessage(&LoraPkt, 0, msg, len);
    }

    // here is where we send the packet out
    Lora.LoraPrepSend(&LoraPkt);
    sched.WaitUntil    (theNode.nTimeslot & 0xff, 0, 0, TaskHandles[LoRaSendTaskNdx]);
    // send the packet during our slot
    sched.PrintTime("Sending ");
    Lora.LoraSend(&LoraPkt);
    // DumpByteData("sent Upstream ", LoraPkt.Buf, LoraPkt.nSize);


    sched.DoUI();
  #if 1
  if (theNode.role!=ROLE_BEACON)
  {
   printf("Hdop = %f %f\n",(float)gps.hdop.value(),theNode.configMinHdop);
   if ((gps.hdop.value()>theNode.configMinHdop) ||gps.hdop.value()==0)
      DOHFRange();
     LoraHF.SetSleep(1);
  }
  #endif

    disp_div++;
    if (disp_div>=theNode.configRefresh)
      {
      sched.RequestScreenRefresh();
      disp_div=0;
      }
  }
}

void NodeTaskSpy(void *parameter)
{
  LORA_PKT LoraPkt;
  int nRSSI;
  int i;

  printf("Started task \n");
  while (1)
  {
    vTaskDelay(1);
    if (Lora.ReceivePacket(&LoraPkt) == 0)
      continue;
    //    theNode.rxRSSI=LoRa.packetRssi();
    printf("\n\n");
    sched.PrintTime();

    theNode.beaconReceived = 0;
    theNode.ProcessRxPack(&LoraPkt);

    // if we got a beacon we can try to join

    if (theNode.beaconReceived)
    {
      printf("Got beacon ----------------> len = %d\n", LoraPkt.nSize);
      //   sched.PrintTime();
      sched.ResetTime(CalcPacketTime(LoraPkt.nSize),theNode.rxEpoch);
      sched.PrintTime();
    }
    DumpByteData("Rx:", LoraPkt.Buf, LoraPkt.nSize);
  }
}

/* Find the best channel to try to connect to */
/* we could store the timing offsets as well so as not to waste an epoch */
int GWScan()
{
  int bestCh = -1;
  float bestSnr = -1000;
  char retVal = 0;
  int uidiv=0;

  LORA_PKT LoraPkt;
  printf("Num bands %d\n",theNode.bands.size());
  for (auto freq : theNode.bands)
  {
    printf("Scanning Channel %d n", freq);
    Lora.SetFrequency (freq);
    sched.ResetTime(0);
    sched.super_epoch_size = 2;
    //wait for 1 beacon +
    sched.SetEventTime(20, 2, 0);
    while (!sched.EventExpired())
    {
      vTaskDelay(1);
      if (uidiv++==100)
          {
          sched.DoUI();
          uidiv=0;
          }
      if (Lora.ReceivePacket(&LoraPkt) == 0)
        continue;
      theNode.beaconReceived = 0;
      theNode.ProcessRxPack(&LoraPkt);
      if (theNode.beaconReceived)
      {
        printf("Found beacon on ch %d rssi %d snr %f\n", freq, LoraPkt.rssi, LoraPkt.SNR);
        if (LoraPkt.SNR > bestSnr)
        {
          bestSnr = LoraPkt.SNR;
          bestCh = freq;
          printf("Best SNR %f bestCH %d\n", bestSnr, bestCh);
        };
        break;
      }
    }
  }
  sched.super_epoch_size = 1;
  if (bestCh < 0)
  {
    bestCh = 0;
    printf("Gateway not found defaulting to %d\n", bestCh);
    retVal = 0;
  }
  else
  {
    //        LoRa.setFrequency(BandFrequencyTable[bestCh]);
    printf("Gateway found at %d\n", bestCh);
    retVal = 1;
  }
 /* sched.disp_ok = 1;
  sched.disp_now = 1;
  while (sched.disp_now)
    vTaskDelay(10);
    */
  return retVal;
}

#define JOIN_TIMOUT 7000

int NodeDoJoin()
{
  LORA_PKT LoraPkt;
  int i;
  int t = clock();

  int timeOut = JOIN_TIMOUT;
  printf("Started Join \n");
  while (1)
  {
    vTaskDelay(1);
    if (timeOut-- == 0)
    {
      printf("Join Failed  %d\n", clock() - t);
      return 0;
    }
    if (Lora.ReceivePacket(&LoraPkt) == 0)
      continue;

    theNode.beaconReceived = 0;
    theNode.ProcessRxPack(&LoraPkt);
    if (theNode.beaconReceived)
        sched.ResetTime(CalcPacketTime(LoraPkt.nSize));

    // if we got a beacon we can try to join
    if (theNode.connectState != NodeActive)
      if (theNode.beaconReceived)
      {
        theNode.BuildJoinMessage(&LoraPkt, theNode.rxGatewayID, LoraPkt.rssi);
        sched.PrintTime("Got beacon Join");

        // lets send it out during timeslotFREE_FOR_ALL_TS
        Lora.LoraPrepSend(&LoraPkt);
        sched.WaitUntil(FREE_FOR_ALL_TS, 0, 0);
        Lora.LoraSend(&LoraPkt);
        sched.PrintTime("Sent Join");

//        DumpByteData("Sending Join Req:\n", LoraPkt.Buf, LoraPkt.nSize);
      }
    printf("Connect state = %d\n", theNode.connectState);
    theNode.rxGatewayID = -1;
    if (theNode.connectState == NodeActive)
    {
      sched.PrintTime("Going active");
      return 1;
    }
  }
}


#define DISPLAY_UPDATE_TIME 20*CLOCKS_PER_SEC
void NodeTask(void *parameter)
{
  LORA_PKT LoraPkt;
  int nRSSI;
  int i;
  int last_display_update=0;
  int cnt=0;

  printf("Started task \n");
#define SLEEP_BETWEEN_SCANS 30000
   if  (theNode.role==ROLE_BEACON)
       {
          vTaskDelay(1);
          sched.spi_take(200,0);
          printf("got spi\n");
          LoraHF.DoRangeSlave();
          printf ("%d\n",cnt++);
          sched.spi_give();
       }
  
  while(0)
  {
    DOHFRange();
    delay(1000);
    while (theNode.stop == 1)
      vTaskDelay(1000);

  }
  while (1)
  {
    // scan for a gateway
    SetStatusString("Scanning..");
    //    LedSide.set_color(COL_RED);
    while ((GWScan() == 0))
       {
         if ((sched.clock()-last_display_update)>DISPLAY_UPDATE_TIME)
           {
             sched.UpdateDisplay();
             last_display_update=sched.clock();
           }
         Lora.SetPowerMode(POWER_OFF_RETAIN);
         if (theNode.role!=ROLE_BEACON)
           LoraHF.SetSleep(1);
         gps.Enable(0);
         sched.light_sleep(SLEEP_BETWEEN_SCANS);
         gps.Enable(1);
         Lora.SetPowerMode(POWER_ON);
       }
 
    SetStatusString("Joining ..");
    if (NodeDoJoin() == 0)
      continue;
    SetStatusString("Connected ..");
    //  LedSide.set_color(COL_GRN);
    OpMode();
    sched.disp_request=1;
    while (theNode.stop == 1)
      vTaskDelay(1000);

  }

}

#define VERSION "1.2.2"


void ScreenPowerOff()
{
  disp.fillScreen(0);
  disp.setFont(&FreeSansBold9pt7b);
  disp.setCursor(0, 15);
  disp.println("Powered Off");
  //disp.println(theNode.line2);
    disp.changeImage();

}

/*************************************************************************
   DisplayTask
    Display Driver task
 ************************************************************************/
void ScreenMain()
{
  QRCode qrcode;
  uint8_t qrcodeData[qrcode_getBufferSize(3)];
  int start_x = 120;
  int start_y = 38;

  disp.fillScreen(0);
  if (theNode.connectState!=0)
  {
  disp.drawBitmap(120, 5, bitmapAntenna, 18, 18, 1, 0);
  disp.setFont();
  disp.setCursor(140, 15);
  disp.println(String(theNode.nRSSI));
  disp.setCursor(140, 2);
  disp.print("ID:");
  disp.println(String(theNode.nGatewayId));
  
  }
  disp.setCursor(170, 10);
  disp.println(String(VERSION));

#define QSIZE 2
  qrcode_initText(&qrcode, qrcodeData, 2, 0, GetMacString(theNode.myMac));
  for (uint8_t y = 0; y < qrcode.size; y++)
  {
    for (uint8_t x = 0; x < qrcode.size; x++)
    {
      if (qrcode_getModule(&qrcode, x, y))
      {
        disp.fillRect(start_x + x * QSIZE, start_y + y * QSIZE, QSIZE, QSIZE, 1);
      }
    }
  }
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL ); Serial.print( "Memory: " ); Serial.print(uxHighWaterMark ); Serial.print( " "); Serial.println( esp_get_free_heap_size() );
     
  float soc=charge_mon.getRepSOC();
  
  disp.setFont(&FreeSansBold9pt7b);
  disp.setCursor(0, 15);
  disp.println(theNode.line1);
  disp.println(theNode.line2);

  if (gps.time.isValid() && gps.location.isValid())
  {
    String ampm;
    int8_t hour = gps.time.hour() ;
    hour += theNode.zoneOffset;
    if (hour<0) hour+=24;
    if (hour>24) hour-=24;

    if (hour>=12)
         ampm="PM";
    else
         ampm="AM";
    if (hour>12) hour-=12;
    if (hour< 10)
      disp.print(F("0"));
    disp.print(hour);
    disp.print(F(":"));
    if (gps.time.minute() < 10)
      disp.print(F("0"));
    disp.print(gps.time.minute());
    disp.print(ampm);
    //disp.print(F(":"));

    /*
    if (gps.time.second() < 10) disp.print(F("0"));
    disp.print(gps.time.second());
    disp.print(F("."));
    if (gps.time.centisecond() < 10) disp.print(F("0"));
    disp.print(gps.time.centisecond());
    */
    disp.println("");
  }
  else
   disp.println("No GPS");
  //disp.println("Smart Tag");
  //disp.drawBitmap(0,30,myBitmap,165,50,1);

 // disp.println("");
  disp.setFont();
  disp.println(GetMacString(theNode.myMac));
  disp.print(" SOC:");
    disp.print(soc,0);
  
}


void ScreenAlert()
{
  disp.fillScreen(0);
  disp.setCursor(0, 10);
  disp.setTextColor(0, 1);

  disp.setFont(&FreeSansBold12pt7b);
  disp.fillRect(0, 0, 200, 40, 1);
  disp.setCursor(10, 33);
  disp.println("ALERT");
  disp.setFont(&FreeSansBold9pt7b);
  disp.setTextColor(1, 0);
  disp.setCursor(10, 60);
  disp.println((char *)theNode.rxData);
}

void ScreenDebug()
{
  char temp[40];
  disp.fillScreen(0);
  disp.setCursor(0, 10);
  disp.setTextColor(1);
  disp.setFont();

  disp.setTextSize(1);
  //disp.println("Smart Tag");
  //disp.drawBitmap(0,30,myBitmap,165,50,1);

  disp.println(GetMacString(theNode.myMac));
#define BAT_HEIGHT 70
  disp.fillRoundRect(170 + 26 / 2 - 5, 10 - 4, 10, 4, 3, 1);

  disp.drawRoundRect(170, 10, 26, BAT_HEIGHT, 5, 1);
  disp.fillRoundRect(170, 10 + BAT_HEIGHT - (BAT_HEIGHT * charge_mon.getRepSOC() / 100), 26, BAT_HEIGHT * charge_mon.getRepSOC() / 100, 5, 1);
  if (charge_mon.getCurrent_mA() > 0)
    disp.drawBitmap(170, 20, chargeBitmap, 26, 47, 1, 0);

  disp.print("RSSI ");
  disp.println(String(theNode.rxRSSI, DEC));
  disp.print("Status ");
  disp.println(String(theNode.connectState, DEC));
  disp.print("ConnId ");
  disp.println(String(theNode.nConnectionId, DEC));

  disp.print("POS ");
  if (gps.location.isValid())
  {
    disp.print(gps.location.lat(), 5);
    disp.print(F(","));
    disp.println(gps.location.lng(), 5);
  }
  else
  {
    disp.println(F("INVALID"));
  }

  disp.print(F("Date/Time: "));
  if (gps.date.isValid())
  {
    disp.print(gps.date.month());
    disp.print(F("/"));
    disp.print(gps.date.day());
    disp.print(F("/"));
    disp.println(gps.date.year());
  }
  else
  {
    disp.println(F("INVALID"));
  }
  disp.print(F(" "));

  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      disp.print(F("0"));
    disp.print(gps.time.hour());
    disp.print(F(":"));
    if (gps.time.minute() < 10)
      disp.print(F("0"));
    disp.print(gps.time.minute());
    disp.print(F(":"));

    /*
    if (gps.time.second() < 10) disp.print(F("0"));
    disp.print(gps.time.second());
    disp.print(F("."));
    if (gps.time.centisecond() < 10) disp.print(F("0"));
    disp.print(gps.time.centisecond());
    */
    disp.println("");
  }
  else
  {
    disp.print(F("INVALID"));
  }
  disp.print("SOC:");
  disp.print(charge_mon.getRepSOC(),0);
  disp.print(" I:");
  disp.print(charge_mon.getAvgCurrent_mA(),0);
  disp.print(" E:");
  disp.println(charge_mon.getTTE(),0);

  disp.println(LoraHF.hf_locs[0].dist);
  theNode.rxData[10] = 0;
  //disp.setTextSize(2);
  disp.setFont(&FreeSansBold9pt7b);
  disp.println((char *)theNode.rxData);
}

void DisplayBeacon(void)
{
  disp.fillScreen(0);
  disp.setCursor(0, 10);
  disp.setTextColor(1);
  disp.setFont();
  
   disp.setTextSize(1);
  //disp.println("Smart Tag");
  //disp.drawBitmap(0,30,myBitmap,165,50,1);

  disp.print("Beacon: ");
  disp.println(theNode.nRSSI);
  disp.println(GetMacString(theNode.myMac));

  disp.println(theNode.rangingAddress,DEC);
  disp.print(LoraHF.GetFrequencyError());
  disp.print(" HFRSSI: ");
  disp.print(LoraHF.GetRssi());
  disp.print("POS ");
  if (gps.location.isValid())
  {
    disp.print(gps.location.lat(), 5);
    disp.print(F(","));
    disp.println(gps.location.lng(), 5);
  }
  else
  {
    disp.println(F("INVALID"));
  }

  disp.print(F("Date/Time: "));
  if (gps.date.isValid())
  {
    disp.print(gps.date.month());
    disp.print(F("/"));
    disp.print(gps.date.day());
    disp.print(F("/"));
    disp.println(gps.date.year());
  }
  else
  {
    disp.println(F("INVALID"));
  }
  disp.print(F(" "));

  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10)
      disp.print(F("0"));
    disp.print(gps.time.hour());
    disp.print(F(":"));
    if (gps.time.minute() < 10)
      disp.print(F("0"));
    disp.print(gps.time.minute());
    disp.print(F(":"));

    /*
    if (gps.time.second() < 10) disp.print(F("0"));
    disp.print(gps.time.second());
    disp.print(F("."));
    if (gps.time.centisecond() < 10) disp.print(F("0"));
    disp.print(gps.time.centisecond());
    */
    disp.println("");
  }
  else
  {
    disp.print(F("INVALID"));
  }
  disp.print("SOC:");
  disp.print(charge_mon.getRepSOC(),0);
  disp.print(" I:");
  disp.print(charge_mon.getAvgCurrent_mA(),0);
  disp.print(" E:");
  disp.println(charge_mon.getTTE(),0);
}
  int alert_level = 0;

void DisplayNode(void)
{
  char temp[20];

  if (strstr((const char *)theNode.rxData, "ALERT1") != 0)
  {
    LedSide.set_color(COL_RED);
    LedTop.set_color(COL_RED);
    alert_level = 1;
  }
  if (strstr((const char *)theNode.rxData, "ALERT2") != 0)
  {
    LedSide.set_color(COL_MAG);
    LedTop.set_color(COL_MAG);
    alert_level = 2;
  }
  if (strstr((const char *)theNode.rxData, "ALERT3") != 0)
  {
    LedSide.set_color(COL_GRN);
    LedTop.set_color(COL_GRN);
    alert_level = 3;
  }
  if (alert_level > 0)
  {
    ScreenAlert();
    if (buttons.pressed)
    {
      buttons.pressed = 0;
      theNode.rxData[0] = 0;
    }
  }
  else
    ScreenMain();
}


void DisplayTask(void *parameter) // Dispay Task
{
  const char *FreeTable[] = {"Free", "Active"};
  int next_sleep = 50;
#define UPDATE_REDRAW 10
  int update_count = UPDATE_REDRAW;
  printf("Display Task started\n");
  
  while (1)
  {
    while (sched.disp_now==0)
       vTaskDelay(10);
    if (theNode.otaMode)
        {
         vTaskDelay(10);
         continue;

        }
    sched.RequestNoSleep("Display",0x1);
    if (theNode.role==ROLE_NORMAL)
      {
      if (theNode.mode==MODE_NORMAL)
        DisplayNode();
      else
        if (theNode.mode==MODE_DEBUG)
      ScreenDebug();
    
      }
    else
      DisplayBeacon();
    int  t= clock();
    printf("doing display\n");
    disp.changeImage();
//     disp.updateImage();
    printf("display done  %d\n",clock()-t);
    sched.ReleaseNoSleep(0x1);
    sched.disp_now=0;
  }
}

void DoWaitBusy(void)
{
  //printf ("Test busy %d \n",IOreadRegister(0));
  return;
  while (IOreadRegister(0) & BIT_BUSY)
  {
    printf("waiting ..\n");
  }
}

void kbhit()
{
  while(!Serial.available());

  while(Serial.available())
  {
  // get the character
  char ch = Serial.read();

  // echo the character to serial monitor or PuTTY
  Serial.write(ch);
  }
}

#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"

void do_snd(int f);



void WireTake(void)
{
  sched.wire_take(10);
}

void WireGive(void)
{
  sched.wire_give();
}



#define MIN_SOC_SHUTDOWN 5
void  TestShutDown()
{
  int soc = charge_mon.get_SOC();
  printf("Testing shutdown %d  %d\n",soc,charge_mon.GetUserMem());
  if (soc<MIN_SOC_SHUTDOWN)
    {
      int shutdownDisplayed = charge_mon.GetUserMem();
      if (shutdownDisplayed)
         LPTest();
      else
         {
           ScreenPowerOff();
           shutdownDisplayed=1;
           charge_mon.WriteUserMem(1);
           LPTest();
         }
    }
  else
    charge_mon.WriteUserMem(0);
}


void setup()
{
  // put your setup code here, to run once:

  //Serial.begin(921600);
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n\nBooting Node");
  Serial.printf("Version %s %s\n",__DATE__,__TIME__);
  sched.Init();

  // i2c setup
  Wire.begin(23, 19);
  Wire.setClock(100000);
  Wire.setTimeOut(100);

  IOwriteRegister(TCA6408A_CONFIG, ~(BIT_RESETn | BIT_MTR));
  IOwriteRegister(TCA6408A_OUTPUT,0);
  DoReset();
  delay(100);
  
  pinMode(34, INPUT);
  
  pinMode(PIN_AUDIO_SD,OUTPUT);
  digitalWrite(PIN_AUDIO_SD,LOW);
  // initialize the chip selects
  pinMode(CS_1, OUTPUT);
  pinMode(CS_2, OUTPUT);
  pinMode(CS_3, OUTPUT);
  pinMode(CS_4, OUTPUT);
  digitalWrite(CS_1, HIGH);
  digitalWrite(CS_2, HIGH);
  digitalWrite(CS_3, HIGH);
  digitalWrite(CS_4, HIGH);

  SPI.begin(SCK, MISO, MOSI, -1);
  SPIFFS.begin();
  if (!SPIFFS.exists("/formatComplete.txt")) 
    {
    Serial.println("Please wait 30 secs for SPIFFS to be formatted");
    SPIFFS.format();
    Serial.println("Spiffs formatted");
    File f = SPIFFS.open("/formatComplete.txt", "w");
    if (!f) {
        Serial.println("file open failed");
    } else {
        f.println("Format Complete");
        f.close();
        }
     }

  charge_mon.begin();
  charge_mon.forcedExitHiberMode();
  disp.setup();
  TestShutDown();
  
  LoraHF.setup();
  
  gps.setup();

  Lora.setup();
   //LPTest();


  buttons.setup();

  printf("chargemon %f ",charge_mon.getCurrent_mA());
  theNode.NodeInit();


  GetMac(theNode.myMac);
  {
    void loadConfiguration(const char *filename);
    loadConfiguration("/config.json");
  }

  LedSide.setup();
  LedTop.setup();
  LedSide.SetPWM(theNode.ledPWM);
  LedTop.SetPWM(theNode.ledPWM);

  LoraHF.SetRangingCalibration(theNode.hfDelayCal);
  //theNode.rangingAddress=0x32100000;    
  LoraHF.SetRangingAddress(theNode.rangingAddress+RANGE_ADDRESS_BASE);
  //LoraHF.SetRangingAddress(0x32100000);
 
  motion.setup();

  while (0) 
  {
    DOHFRange();
    delay(100);
  }
   altimeter= new LPS25HBSensor (&Wire);
  altimeter->Enable();
  altimeter->SetODR(25);
while(0)
{

         float press;
        sched.wire_take(10);
        altimeter->GetPressure(&press);
        sched.wire_give();
        printf("\nPress = %3.3f\n",press);
}
  xTaskCreatePinnedToCore(DisplayTask, "Display", 4000, NULL, DisplayPriority, &TaskHandles[DisplayHandle], CORE_1);

  // display initial screen
  sched.UpdateDisplay();
  //LPTest();

  LedSide.set_color(COL_RED);
  LedTop.set_color(COL_RED);
  delay(500);
  LedSide.set_color(COL_GRN);
  LedTop.set_color(COL_GRN);
  do_snd(4000);
  delay(500);
  LedSide.set_color(COL_BLUE);
  LedTop.set_color(COL_BLUE);
  do_snd(2000);
  delay(500);
  do_snd(0);

  STdigitalWrite(PIN_MTR,HIGH);

  sched.UpdateDisplay();
  LedSide.set_color(0);
  LedTop.set_color(0);
  STdigitalWrite(PIN_MTR,0);

  xTaskCreatePinnedToCore(NodeTask, "LoRaSend", 11000, NULL, LoRaSendPriority, &TaskHandles[LoRaSendTaskNdx], CORE_1);
  xTaskCreatePinnedToCore(gps.GPSTask, "GPS", 2000, (void *)&gps, GPSPriority, &TaskHandles[GPSHandle], CORE_1);
  xTaskCreatePinnedToCore(buttons.ButtonTask, "Button", 1800, (void *)&buttons, ButtonPriority, &TaskHandles[ButtonHandle], CORE_1);
}

void loop()
{
  static int last_run=sched.clock();
  // put your main code here, to run repeatedly:
  std::list<uint8_t> debug_mode({BUTTON_LEFT,BUTTON_BOTTOM,BUTTON_RIGHT});
  std::list<uint8_t> beacon_mode({BUTTON_LEFT,BUTTON_TOP,BUTTON_RIGHT});
//printf("In loop\n");
  xSemaphoreTake(sched.user_sem,100);
  if ((sched.clock()-last_run)<100)
     return;
//  printf("Buttons  %d %d\n",sched.clock(),sched.clock()-last_run);
  last_run=sched.clock();
  buttons.ProcessInput();
  if (buttons.key_list==debug_mode)
     {
       printf("Should enter debug\n");
       sched.RequestScreenRefresh();
       theNode.mode=MODE_DEBUG;
       buttons.key_list.clear();
     }
  if (buttons.key_list==beacon_mode)
    {
      theNode.role=ROLE_BEACON;
       buttons.key_list.clear();
    }
  if (buttons.long_push == BUTTON_TOP)
    {
      int x;
        LedSide.set_color(0); 
      STdigitalWrite(PIN_MTR,HIGH);
      sched.RequestNoSleep("Sound",0x2);
      for (x=2000;x<6000;x+=200)
      {
      do_snd(x);
      printf ("Doing sound %d\n",x);
      delay(100);
      }
      LedSide.set_color(0); 
      STdigitalWrite(PIN_MTR,0);
      do_snd(0);
      buttons.long_push=0;
      sched.ReleaseNoSleep(0x2);
      theNode.panic=1;
      strcpy((char *)theNode.rxData,"ALERT1Panic");
    }
  if (buttons.long_push == BUTTON_LEFT)
  {
       if (theNode.panic)
         {
        LedSide.set_color(0);
      LedTop.set_color(0);
      theNode.rxData[0] = 0;
      alert_level=0;
      theNode.panic=0;
      sched.RequestScreenRefresh();
      }
  }
  if (buttons.long_push == BUTTON_BOTTOM)
  {
    printf("lp = %d\n", buttons.long_push);
    buttons.long_push = 0;
    {
      void ota_setup(void);
      theNode.stop = 1;
       sched.RequestNoSleep("Ota",0x20);
      delay(1000);
      ota_setup();
    }
  }


if (1)
{
  // this code keeps the soc monitor allive
  charge_mon.getCurrent_mA();
  charge_mon.getAvgCurrent_mA();
  charge_mon.getRepSOC();
  charge_mon.getVoltageCell();
}
 //printf("Avg I %3.3f vsum %3.3f\n",charge_mon.getAvgCurrent_mA(),motion.vsum);
motion.Process();

if (motion.IsActive())
   printf("We have motion %3.3f %3.3f  %d\n",motion.vsum,motion.th,sched.in_wait);
//if (Serial.read()>0)
   {
     uint8_t IOIntReset(void);
   //printf("bat infor I %3.3f avgI %3.3f SOC %3.3f gps: %d %d\n",charge_mon.getCurrent_mA(),charge_mon.getAvgCurrent_mA(),charge_mon.getRepSOC(),gps.state
    //, digitalRead(34));
  if (digitalRead(34)==0)
     printf("int reset 0x%x\n",IOIntReset());
   }
  
  if (charge_mon.getCurrent_mA()>-1)
     {
      if (charge_mon.getRepSOC()>=99)
        LedSide.set_color(COL_GRN);
      else 
        LedSide.set_color(COL_RED); 
     }
  else
  {
    if (alert_level==0)
      LedSide.set_color(0); 
  }
 
  if (alert_level > 0)
  {
    uint8_t col;
    static uint8_t alrt_div=0;

    switch (alert_level)
    {
      case 1:col=COL_RED;break;
      case 2:col=COL_MAG;break;
      case 3:col=COL_GRN;break;
    }
    alrt_div++;
    if (alrt_div==3)
      {
      LedSide.set_color(col);
      LedTop.set_color(col);
      STdigitalWrite(PIN_MTR,0);
      
      do_snd(2000);
      sched.RequestNoSleep("Sound alert",0x8);

      }
    if (alrt_div==5)
      do_snd(00);

    if (alrt_div==6)
      {
      LedSide.set_color(0);
      LedTop.set_color(0);
      STdigitalWrite(PIN_MTR,HIGH);
      alrt_div=0;
      do_snd(00);
      sched.ReleaseNoSleep(0x8);
      }

    if (alert_level && buttons.pressed && theNode.panic==0)
    {
      STdigitalWrite(PIN_MTR,0);
      LedSide.set_color(0);
      LedTop.set_color(0);
      theNode.rxData[0] = 0;
      alert_level=0;
      sched.RequestScreenRefresh();
      alrt_div=0;
      do_snd(0);
    }
  }
  //sched.delayRTC(100); // wait for a second
     xSemaphoreGive(sched.user_sem_done);
}
 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"



extern "C" void app_main()
{
  esp_err_t ret;
// Initialize NVS.


ret = nvs_flash_init_partition("nvs");
ESP_LOGE("BOOT", "%s - nvs_flash_part() = %x", __func__, ret);
ret = nvs_flash_init();
ESP_LOGE("BOOT", "%s - nvs_flash_init() = %x", __func__, ret);
    initArduino();
    setup();
    //do your own thing
    vTaskDelay(200);
    printf("Starting loop\n");
  while(1)
     loop();
}