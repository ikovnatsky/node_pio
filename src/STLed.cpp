#include "Arduino.h"
#include "STLed.h"
#include "Wire.h"
void WireTake(char *who = NULL);
void WireGive(void);

void STLed::setup(void)
{
  //set Blink 1 frequency
  RegWrite(PSC0, 1);
  RegWrite(PSC1, 1);
  // set PWM1 to 50 %
  RegWrite(PWM0, 50);
  RegWrite(PWM1, 5);

  //set LED 8 to Blink 1
}

void STLed::SetPWM(int val)
{
  RegWrite(PWM0, val);
}

uint8_t STLed::get_col(void)
{
  return cur_col;
}

void STLed::set_color(uint8_t col)
{
  uint8_t val;
  uint8_t vals[3];
  //  printf("Setting color %d\n",col);

  cur_col = col;
  vals[0] = vals[1] = vals[2] = 0;
  if (col & COL_RED)
  {
    vals[0] |= 0b10000010; //
    vals[1] |= 0b00100000;
    vals[2] |= 0b00001000;
  }
  if (col & COL_GRN)
  {
    vals[0] |= 0b00001000; //
    vals[1] |= 0b10000010;
    vals[2] |= 0b00100000;
  }
  if (col & COL_BLUE)
  {
    vals[0] |= 0b00100000; //
    vals[1] |= 0b00001000;
    vals[2] |= 0b10000010;
  }
  // vals[3]=0;
  /*
   RegWrite(  LS0,vals[0]);
   RegWrite(  LS1,vals[1]);
   RegWrite(  LS2,vals[2]);
   RegWrite(  LS3,vals[3]);*/
  ColWrite(vals);
}

void STLed::ColWrite(uint8_t *c)
{
  uint8_t buff[4];

  WireTake("Led");
  delay(1);
  buff[0] = LS0 | 0x10; // auto inc
  buff[1] = c[0];
  buff[2] = c[1];
  buff[3] = c[2];
  int error = Wire.writeTransmission(address, buff, 4);
  /*
      Wire.beginTransmission(address); //open communication with 
    Wire.write(LS0);  
    Wire.write(*c++); 
    Wire.write(*c++); 
    Wire.write(*c++); 
    int error = Wire.endTransmission();
  */
  delay(1);
  if (error)
  {
    printf("LED error write %d \n", error);
    Wire.flush();
  }

  WireGive();
}
void STLed::RegWrite(uint8_t addra, uint8_t v)
{
  WireTake("Led");
  delay(1);
  Wire.beginTransmission(address); //open communication with
  Wire.write(addra);
  Wire.write(v);
  int error = Wire.endTransmission();
  delay(1);
  if (error)
  {
    printf("LED error write %d \n", error);
    Wire.flush();
  }

  WireGive();
}
