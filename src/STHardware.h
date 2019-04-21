#define V2

#define LORA_SS   13
#define LORA_TXEN 26
#define LORA_RXEN 27

#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    4   // GPIO19 -- SX127x's MISO
#define MOSI    2   // GPIO27 -- SX127x's MOSI
//#define SS      13   // GPIO18 -- SX127x's CS
#define RST    -1   // GPIO14 -- SX127x's RESET
#define DI00    -1   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define GPS_RX_PIN 38
#define GPS_TX_PIN 21

#define CS_DISPLAY 12
#define EXPANDER_IO_PIN_OFFSET 0x40
#define PIN_DISCHARGE   EXPANDER_IO_PIN_OFFSET+0
#define PIN_BUTTONS     35

#define PIN_AUDIO_SD    22

#define CS_1 12
#define CS_2 13
#define CS_3 14
#define CS_4 18


#ifdef V1
#define PIN_MTR  EXPANDER_IO_PIN_OFFSET + 5
#define BIT_RESETn  4
#define BIT_BUSY    2
#define BIT_MTR    0x20
#define BIT_DISCHARGE  1
#else
#define BIT_MTR    0x4000
#define BIT_SHUTDOWN 0x1000
#define BIT_RESETn  4
#define BIT_BUSY    2

#define PIN_MTR  EXPANDER_IO_PIN_OFFSET + 14
#define PIN_BUTTON_1 EXPANDER_IO_PIN_OFFSET+8
#define PIN_BUTTON_2 EXPANDER_IO_PIN_OFFSET+9
#define PIN_BUTTON_3 EXPANDER_IO_PIN_OFFSET+10
#define PIN_BUTTON_4 EXPANDER_IO_PIN_OFFSET+15
#define PIN_SHUTDOWN  EXPANDER_IO_PIN_OFFSET+12
#endif