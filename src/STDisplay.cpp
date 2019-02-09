
#include <Arduino.h>
#include "EpaperDriver.hpp"
//#include "Adafruit_GFX.h"
#include "/home/me/Documents/PlatformIO/Projects/node_pio/lib/Adafruit_GFX_Library/Adafruit_GFX.h"
#include "STHardware.h"
#include "STDisplay.h"

#define  MAX_WIDTH  200
#define MAX_HEIGHT  96

static uint8_t prevImage[MAX_WIDTH * MAX_HEIGHT / 8] = {};
static uint8_t image[MAX_WIDTH * MAX_HEIGHT / 8];
static EpaperDriver epd(EpaperDriver::Size::EPD_2_00_INCH, prevImage);


void STDisplay::setup(void)
{
    epd.panelOnPin = 3;
  epd.borderControlPin = -1;
  epd.dischargePin = PIN_DISCHARGE;
  epd.resetPin = -1;
  epd.busyPin = -1;
  epd.chipSelectPin = CS_DISPLAY;
  epd.setFrameTime(300);
  pinMode(CS_DISPLAY,OUTPUT);
  digitalWrite(CS_DISPLAY,HIGH);
  int width = epd.getWidth();
  int height = epd.getHeight();
  int checkerSize = 10;
  

  
}

void STDisplay::changeImage()
{
  epd.changeImage(getBuffer());
}
void STDisplay::updateImage()
{
  epd.updateImage(getBuffer());

}