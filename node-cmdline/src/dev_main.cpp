#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "cmd_line.h"

#include "sdkconfig.h"

extern "C" void app_main()
{
    printf("ESP32 node ready.\n");

    // start console
    cmdLineInit();

    xTaskCreate(cmdLineTaskSerialPrompt, "cmdline", 65536,NULL,5,NULL );
} // end app_main