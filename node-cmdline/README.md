# Node Command Line

## Purpose
This project provides a command line interface on the ESP32's serial port. While conducting FCC tests, the operator changes LoRa radio settings through this interface.

## Installation

  1. Open and compile with platform.io.
  1. Flash to an ESP32.
  1. Reset the device.
  1. Open a serial terminal.
  
 Serial terminal example for Linux or Mac:
  
    screen /dev/ttyUSB0 115200
    
## Usage
On reset, the serial port should display something like:

    This is the node console.
    Type 'help' to get the list of commands.
    node>

Enter `help` to see available commands and their parameters. 

Change settings on the SX1262 LoRa Radio with the `lora` command, and on the SX1280 LoRaHF Radio with the `lorahf` command.

Each command accepts a settings name followed by an integer argument. Refer to the `help` command for details.

For example, to set the LoRaHF bandwidth to 800 kHz, do:

    node> lorahf bw 800
    setting LoRaHF bandwidth to 800 kHz.


## Known Issues and ToDos

  * This implementation is a stub for review purposes. None of the commands actually affect the radio. 
  * On some terminals, the underlying `esp-console` library introduces arbitrary line breaks into help texts.