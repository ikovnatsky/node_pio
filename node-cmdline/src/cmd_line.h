/*************************************************** 
  Compliance test command line interface

  File: cmd_line.h
  Version: 1.0
  Date: April 2019

  Written by christian@eeproto.com for David Freed
 ****************************************************/

#ifndef __CMDLINE_H__
#define __CMDLINE_H__

#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"

#include <sx126x-hal.h>

void cmdLineInit();
void cmdLineTaskSerialPrompt(void *pvParameter);
void loraInit();
void loraSend();

#define LORA_CS   13
#define LORA_TXEN 26
#define LORA_RXEN 27

#define LORAHF_CS 14
#define DISPLAY_CS 12

#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    4   // GPIO19 -- SX127x's MISO
#define MOSI    2   // GPIO27 -- SX127x's MOSI
//#define SS      13   // GPIO18 -- SX127x's CS
#define RST    -1   // GPIO14 -- SX127x's RESET
#define DI00    -1   // GPIO26 -- SX127x's IRQ(Interrupt Request)

#define MAX_LORA_PKT 80
#define CMD_LINE_UART_NUMBER UART_NUM_0

#endif