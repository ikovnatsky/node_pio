#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "cmd_line.h"

#include "sdkconfig.h"

int g_wait_delay=0;
void DoWaitBusy(void)
{
  return;
}

void loop()
{
}

//extern "C" void app_main()
void setup()
{
    printf("ESP32 node ready.\n");
    // start console
    cmdLineInit();

    // initialize lora modules
    loraInit();

    xTaskCreate(cmdLineTaskSerialPrompt, "cmdline", 65536,NULL,5,NULL );
} // end app_main