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
#define BIT_RESETn  4
#define BIT_BUSY    2
#define BIT_MTR    0x20
#define BIT_DISCHARGE  1
#define EXPANDER_IO_PIN_OFFSET 0x40
#define PIN_DISCHARGE   EXPANDER_IO_PIN_OFFSET+0
#define PIN_MTR  EXPANDER_IO_PIN_OFFSET + 5
#define PIN_BUTTONS     35

#define PIN_AUDIO_SD    22


