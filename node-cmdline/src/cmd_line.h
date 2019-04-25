#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

void cmdLineInit();
void cmdLineTaskSerialPrompt(void *pvParameter);
