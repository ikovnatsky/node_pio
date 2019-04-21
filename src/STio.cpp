#include "Arduino.h"
#include <Wire.h>
#include "STHardware.h"
#include "STio.h"

void WireTake(char *who=NULL);
void WireGive(void);

uint16_t set_val =0;
void IOSetup(void)
{
       Wire.begin(23,19);
       Wire.setClock(100000);
}



#ifdef V1
bool IOwriteRegister(const uint8_t register_addr, const uint8_t value) {
    //send write call to sensor address
    //send register address to sensor
    //send value to register
    bool write_status = 0;
    Wire.beginTransmission(EXPANDER_ADDRESS); //open communication with 
    Wire.write(register_addr);  
    Wire.write(value); 
     Wire.endTransmission();
  //if (error)
   // printf("IO error write reg %d \n",error);

    return write_status; //returns whether the write succeeded or failed
}

uint8_t IOreadRegister(const uint8_t register_addr) {
    //call sensor by address
    //call registers
    uint8_t data = 0;

    Wire.beginTransmission(EXPANDER_ADDRESS); 
    Wire.write(register_addr); 
    Wire.endTransmission(); 

    Wire.requestFrom((int)EXPANDER_ADDRESS, 1);

    while(Wire.available()) {
        data = Wire.read();    // receive a byte as character
    }

    return data; //return the data returned from the register
}


uint8_t IOreadRegister2(const uint8_t register_addr) {
    //call sensor by address
    //call registers
    uint8_t data = 0;

    Wire.beginTransmission(0x36); 
    Wire.write(register_addr); 
    Wire.endTransmission(); 

    Wire.requestFrom((int)0x36, 1);

    while(Wire.available()) {
        data = Wire.read();    // receive a byte as character
    }

    return data; //return the data returned from the register
}
#endif

#ifdef V2
#define TCA9535_ADDRESS		0x20 		/*!< I2C Address */

/************************** I2C Registers *************************************/
typedef enum {
	TCA9535_INPUT_REG0 =	0x00,		/*!< Input status register */
	TCA9535_INPUT_REG1 =	0x01,		/*!< Input status register */
	TCA9535_OUTPUT_REG0	=	0x02,		/*!< Output register to change state of output BIT set to 1, output set HIGH */
	TCA9535_OUTPUT_REG1	=	0x03,		/*!< Output register to change state of output BIT set to 1, output set HIGH */
	TCA9535_POLARITY_REG0 =	0x04,		/*!< Polarity inversion register. BIT '1' inverts input polarity of register 0x00 */
	TCA9535_POLARITY_REG1 =	0x05,		/*!< Polarity inversion register. BIT '1' inverts input polarity of register 0x00 */
	TCA9535_CONFIG_REG0	=	0x06,		/*!< Configuration register. BIT = '1' sets port to input BIT = '0' sets port to output */
	TCA9535_CONFIG_REG1	=	0x07		/*!< Configuration register. BIT = '1' sets port to input BIT = '0' sets port to output */
} tca9535_reg_t;

bool IOwriteRegister(const uint8_t register_addr, const uint8_t value) {
    //send write call to sensor address
    //send register address to sensor
    //send value to register
    bool write_status = 0;
    uint8_t retry = 3;
    int error= 0;
   while(retry)
   {
      WireTake("io");
      Wire.beginTransmission(TCA9535_ADDRESS); //open communication with 
      Wire.write(register_addr);  
      Wire.write(value); 
    error=   Wire.endTransmission();
    WireGive();
    if (error)
      {
//      printf("IO error write reg %d \n",error);
      }
    else
      break;    
    retry--;
    }
    if (error)
      printf("IO error write reg %d \n",error);
    return write_status; //returns whether the write succeeded or failed
}


uint16_t IOReadInput(void)
{
    uint16_t data = 0;
    int err;
    WireTake("io2");

    while (1)
    {
    Wire.beginTransmission(TCA9535_ADDRESS); 
    Wire.write(TCA9535_INPUT_REG0); 
    err= Wire.endTransmission(); 
       //printf("Erroer reading input \n");
    if (err==0)
      break;
    }

    err= Wire.requestFrom((int)TCA9535_ADDRESS, 2);
    if (err!=2 || (Wire.lastError()!=I2C_ERROR_OK))
       printf("Could not read buttons\n");
    if (Wire.available()) {
        data = Wire.read();    // receive a byte as character
    }
    if (Wire.available()) {
        data |= Wire.read()<<8;    // receive a byte as character
    }
//   Wire.endTransmission(); 
   WireGive();
    return data; //return the data returned from the register

}
uint8_t IOreadRegister(const uint8_t register_addr) {
    //call sensor by address
    //call registers
    uint8_t data = 0;
    WireTake("io3");
    Wire.beginTransmission(TCA9535_ADDRESS); 
    Wire.write(register_addr); 
    Wire.endTransmission(); 

    int err= Wire.requestFrom((int)TCA9535_ADDRESS, 1);
    if (err==0)
       printf("Could not read buttons\n");
    while(Wire.available()) {
        data = Wire.read();    // receive a byte as character
    }
//   Wire.endTransmission(); 
   WireGive();
    return data; //return the data returned from the register
}





void   IOConfig(void )
{
  IOwriteRegister(TCA9535_OUTPUT_REG0,0);
  IOwriteRegister(TCA9535_OUTPUT_REG1,0);
 IOwriteRegister(TCA9535_CONFIG_REG0, ~(BIT_RESETn | BIT_MTR));
  //100 is the shutdown pin
  IOwriteRegister(TCA9535_CONFIG_REG1, (~(BIT_RESETn | BIT_MTR | BIT_SHUTDOWN))>>8);
  
 }
void DoReset()
{
  IOwriteRegister(TCA9535_OUTPUT_REG0,0);
  delay(100);
  IOwriteRegister(TCA9535_OUTPUT_REG0,BIT_RESETn );


  set_val=BIT_RESETn;
 // printf("Reseting \n");
}
uint8_t STdigitalRead(uint8_t a)
{
 uint16_t bit= 1<<(a&(~EXPANDER_IO_PIN_OFFSET));
 //printf("Reading bit %x ",bit);
 if (bit&0xff)
    {
   //   printf( "Val = %d\n",IOreadRegister(TCA9535_INPUT_REG0)&bit);
    return (IOreadRegister(TCA9535_INPUT_REG0)&bit);
    }
 else
  {
     //    printf( "Val = %d\n",IOreadRegister(TCA9535_INPUT_REG1)&(bit>>8));
              return (IOreadRegister(TCA9535_INPUT_REG1)&(bit>>8));

  }

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
       uint16_t bit= 1<<(a&(~EXPANDER_IO_PIN_OFFSET));
       //printf("Reading bit %x ",bit);
            if (v==HIGH)
              {
                  set_val = set_val| (1<<(a&(~EXPANDER_IO_PIN_OFFSET)));
                  if (bit&0xff)
                      IOwriteRegister(TCA9535_OUTPUT_REG0,set_val);
                  else
                      IOwriteRegister(TCA9535_OUTPUT_REG1,set_val>>8);
              }
            else
            {
                set_val = set_val &~(1<<(a&(~EXPANDER_IO_PIN_OFFSET)));
                  if (bit&0xff)
                      IOwriteRegister(TCA9535_OUTPUT_REG0,set_val);
                  else
                      IOwriteRegister(TCA9535_OUTPUT_REG1,set_val>>8);
            }
            printf("Set val = %x\n",set_val);
    }
}


#endif




void STpinMode(uint8_t a, uint8_t v)
{
  //printf("Pinmode sed %d %d\n",a,v);
}

