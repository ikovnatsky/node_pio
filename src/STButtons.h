
#define BUTTON_TOP_V 1826
#define BUTTON_LEFT_V 0
#define BUTTON_RIGHT_V 1130
#define BUTTON_BOTTOM_V 396

#define BUTTON_RANGE 80

#define BUTTON_LEFT 1
#define BUTTON_RIGHT 2
#define BUTTON_TOP  3
#define BUTTON_BOTTOM 4



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
    list<uint8_t> key_list; 
};