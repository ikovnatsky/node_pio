

#include "Arduino.h"
//#include "Adafruit_GFX.h"
#include "/home/me/Documents/PlatformIO/Projects/node_pio/lib/Adafruit_GFX_Library/Adafruit_GFX.h"

class STDisplay : public GFXcanvas1
{
    public:
    STDisplay(void) :GFXcanvas1(200,96){};
    void setup(void);
    void changeImage(void);
    void updateImage(void);
};

