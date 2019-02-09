#define EXPANDER_ADDRESS 0x20
#define TCA6408A_INPUT                  0x00
#define TCA6408A_OUTPUT                 0x01
#define TCA6408A_POLARITY               0x02
#define TCA6408A_CONFIG                 0x03



void IOSetup(void);
void STdigitalWrite(uint8_t a, uint8_t v);
void STpinMode(uint8_t a, uint8_t v);
uint8_t IOreadRegister(const uint8_t register_addr);
bool IOwriteRegister(const uint8_t register_addr, const uint8_t value);
void DoReset();

