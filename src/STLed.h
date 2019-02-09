
#define COL_RED  1
#define COL_BLUE 2
#define COL_GRN  4
#define COL_MAG  (COL_RED |COL_BLUE)
#define COL_ON   0x10

#define INPUT0  0x00
#define INPUT1  0x01
#define PSC0  0x02
#define PWM0  0x03
#define PSC1  0x04
#define PWM1  0x05
#define LS0  0x06
#define LS1  0x07
#define LS2  0x08
#define LS3  0x09

class STLed 
{
    public:
    STLed(uint8_t a){address=a;};
    void setup(void);
    void set_color(uint8_t col);
    void RegWrite(uint8_t addra, uint8_t v);
    void SetPWM(int val);
    uint8_t address;
};

