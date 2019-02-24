#include "Arduino.h"
#include <Wire.h>
#include "STHardware.h"
#include "STio.h"

void WireTake(void);
void WireGive(void);

uint8_t set_val =0;
void IOSetup(void)
{
       Wire.begin(23,19);
       Wire.setClock(100000);
}
bool IOwriteRegister(const uint8_t register_addr, const uint8_t value) {
    //send write call to sensor address
    //send register address to sensor
    //send value to register
    bool write_status = 0;
    WireTake();

    Wire.beginTransmission(EXPANDER_ADDRESS); //open communication with 
    Wire.write(register_addr);  
    Wire.write(value); 
    int error= Wire.endTransmission();
   if (error)
    printf("IO error write reg %d \n",error);

     WireGive();
    return write_status; //returns whether the write succeeded or failed
}

uint8_t IOreadRegister(const uint8_t register_addr) {
    //call sensor by address
    //call registers
    uint8_t data = 0;
    WireTake();

    Wire.beginTransmission(EXPANDER_ADDRESS); 
    Wire.write(register_addr); 
    int error= Wire.endTransmission();
   if (error)
    printf("IO error read reg %d \n",error);

    Wire.requestFrom((int)EXPANDER_ADDRESS, 1);

    while(Wire.available()) {
        data = Wire.read();    // receive a byte as character
    }

     WireGive();
    return data; //return the data returned from the register
}

uint8_t IOIntReset(void)
{
uint8_t v;
v= IOreadRegister(TCA6408A_INPUT);
IOwriteRegister(TCA6408A_INPUT,v);

return (v);
}

uint8_t IOreadRegister2(const uint8_t register_addr) {
    //call sensor by address
    //call registers
    uint8_t data = 0;

    WireTake();

    Wire.beginTransmission(0x36); 
    Wire.write(register_addr); 
    Wire.endTransmission(); 

    Wire.requestFrom((int)0x36, 1);

    while(Wire.available()) {
        data = Wire.read();    // receive a byte as character
    }

    WireGive();
    return data; //return the data returned from the register
}


void DoReset()
{
  IOwriteRegister(TCA6408A_OUTPUT,0);
  delay(100);
  IOwriteRegister(TCA6408A_OUTPUT,BIT_RESETn );
  set_val=BIT_RESETn;
 // printf("Reseting \n");
}


void STpinMode(uint8_t a, uint8_t v)
{
  //printf("Pinmode sed %d %d\n",a,v);
}

void STdigitalWrite(uint8_t a, uint8_t v)
{
  if (a==0xff)
    return;
    if ((a&EXPANDER_IO_PIN_OFFSET)==0)
    {
        digitalWrite(a,v);
    }
  else
    {
            if (v==HIGH)
              {
                  set_val = set_val| (1<<(a&(~EXPANDER_IO_PIN_OFFSET)));
                  IOwriteRegister(TCA6408A_OUTPUT,set_val);
              }
            else
            {
                set_val = set_val &~(1<<(a&(~EXPANDER_IO_PIN_OFFSET)));
             IOwriteRegister(TCA6408A_OUTPUT,set_val);
            }
            printf("Set val = %x\n",set_val);
    }
}

