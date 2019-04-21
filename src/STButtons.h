
#define BUTTON_TOP_V 1826
#define BUTTON_LEFT_V 0
#define BUTTON_RIGHT_V 1130
#define BUTTON_BOTTOM_V 396

#define BUTTON_RANGE 80

#define BUTTON_LEFT 1
#define BUTTON_RIGHT 2
#define BUTTON_TOP  4
#define BUTTON_BOTTOM 8

#define BUTTON_ANY 0xf


#define KEY_LIST_LEN 3

#include <list> 
using namespace std;

class STButtons
{
    public:
    STButtons(){};
    void setup(void);
    uint8_t  pressed;
    uint8_t  pushed;
    uint8_t  long_push;
    int val;
    int down_time;
    static void ButtonTask( void * param);
    void DoTask(void);
    void ProcessInput(void);
    uint8_t GetKey(void);
    uint8_t WaitKey(uint8_t k);
    list<uint8_t> key_list; 
};