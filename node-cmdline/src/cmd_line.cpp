/*************************************************** 
  Compliance test command line interface

  File: cmd_line.cpp
  Version: 1.0
  Date: April 2019

  With help from https://github.com/espressif/esp-idf/tree/master/examples/system/console

  Written by christian@eeproto.com for David Freed
 ****************************************************/

#include "cmd_line.h"

RadioCallbacks_t RadioEvents;
PacketParams_t PacketParams;
SX126xHal lora(1, 2, 3, 4, 5, 6, NC, NC, NC, NC, NC, NC,&RadioEvents);
unsigned char send_buffer[MAX_LORA_PKT];


//
// Command line prompt as a task function
//

void cmdLineTaskSerialPrompt(void *pvParameter)
{

    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char* prompt = "node> ";

    printf("\n"
           "This is the node console.\n"
           "Type 'help' to get the list of commands.\n"
           //"Use UP/DOWN arrows to navigate through command history.\n"
           //"Press TAB to auto-complete commands.\n"
           );

    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "Try screen or Putty.\n");
        linenoiseSetDumbMode(1);
    }

    /* Main loop */
    while(true) {
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char* line = linenoise(prompt);
        if (line == NULL) { /* Ignore empty lines */
            continue;
        }
        /* Add the command to the history */
        linenoiseHistoryAdd(line);

        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(err));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }
} // end serialPrompt


//
// Version command
//
int getVersion(int argc, char **argv)
{
    esp_chip_info_t info;
    esp_chip_info(&info);
    printf("IDF Version:%s\r\n", esp_get_idf_version());
    printf("Chip info:\r\n");
    printf("\tmodel:%s\r\n", info.model == CHIP_ESP32 ? "ESP32" : "Unknow");
    printf("\tcores:%d\r\n", info.cores);
    printf("\tfeature:%s%s%s%s%d%s\r\n",
           info.features & CHIP_FEATURE_WIFI_BGN ? "/802.11bgn" : "",
           info.features & CHIP_FEATURE_BLE ? "/BLE" : "",
           info.features & CHIP_FEATURE_BT ? "/BT" : "",
           info.features & CHIP_FEATURE_EMB_FLASH ? "/Embedded-Flash:" : "/External-Flash:",
           spi_flash_get_chip_size() / (1024 * 1024), " MB");
    printf("\trevision number:%d\r\n", info.revision);
    return 0;
}

void registerVersion()
{
    const esp_console_cmd_t cmd = {
        .command = "version",
        .help = "Get version of chip and SDK",
        .hint = NULL,
        .func = &getVersion,
        .argtable = NULL
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

//
// LoRa commands
//
static struct {
    struct arg_str *setting;
    struct arg_int *arg;
    struct arg_end *end;
} lora_args;

static int processLoraCommand(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &lora_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, lora_args.end, argv[0]);
        return 1;
    }
    if (lora_args.setting->count)
    {
        if (!lora_args.arg->count)
        {
            printf("missing argument\n");
            return -2;
        }
        int arg = lora_args.arg->ival[0];
        const char* setting = lora_args.setting->sval[0];
        if ( strncmp(setting, "freq", 4) == 0 ) {
            printf("setting LoRa frequency to %d Hz.\n", arg);
            //LoRa.SetFrequency(arg);
        } else if ( strncmp(setting, "bw", 4) == 0 ) {
            printf("setting LoRa bandwidth to %d kHz.\n", arg);
        } else if ( strncmp(setting, "send", 1) == 0 ) {
            printf("[Test] sending.\n");
            loraSend();
        } else if ( strncmp(setting, "test", 1) == 0 ) {
            RadioStatus_t rStat = lora.GetStatus();
            printf("[Test] status val=%d res=%d cmd=%d chip=%d cpu=%d\n", 
                rStat.Value, rStat.Fields.Reserved, rStat.Fields.CmdStatus, rStat.Fields.ChipMode, rStat.Fields.CpuBusy);
            RadioError_t e = lora.GetDeviceErrors();
            printf("error val=%d rc64=%d rc13=%d pll=%d adc=%d img=%d xos=%d lock=%d buck=%d\n", 
                e.Value, e.Fields.Rc64kCalib, e.Fields.Rc13mCalib, e.Fields.PllCalib, e.Fields.AdcCalib, e.Fields.ImgCalib, e.Fields.XoscStart, e.Fields.PllLock, e.Fields.BuckStart, e.Fields.PaRamp);
            printf("operating mode %d\n", lora.GetOperatingMode());
            lora.CheckDeviceReady();
            printf("operating mode %d\n", lora.GetOperatingMode());
        } else {
            printf("unknown setting %s\n", setting);
        }        
    }
    else
    {
        printf("missing command\n");
        return -1;
    }
    fflush(stdout);
    return 0;
}

static int processLoraHFCommand(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &lora_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, lora_args.end, argv[0]);
        return 1;
    }
    if (lora_args.setting->count)
    {
        if (!lora_args.arg->count)
        {
            printf("missing argument\n");
            return -2;
        }
        int arg = lora_args.arg->ival[0];
        const char* setting = lora_args.setting->sval[0];
        if ( strncmp(setting, "freq", 4) == 0 ) {
            printf("setting LoRaHF frequency to %d Hz.\n", arg);
        } else if ( strncmp(setting, "bw", 2) == 0 ) {
            printf("setting LoRaHF bandwidth to %d kHz.\n", arg);
        } else {
            printf("unknown setting %s\n", setting);
        }        
    }
    else
    {
        printf("missing command\n");
        return -1;
    }
    fflush(stdout);
    return 0;
}

static void registerLoraCommand()
{
    lora_args.setting =
        arg_strn(NULL, NULL, "<setting>", 0, 1,
                 "Change the LoRa setting named <setting>."
                 );
    lora_args.arg =
        arg_intn(NULL, NULL, "<arg>", 0, 1,
                 "Integer argument the settings.");
    lora_args.end = arg_end(10);

    const esp_console_cmd_t cmd = {
        .command = "lora",
        .help = "Change SX1262 LoRa Radio Settings.\n"
                 "  Settings accept one integer argument <arg>.\n"
                 "  Supported settings are:\n"
                 "      freq: Transmit at RF frequency <arg> Hz.\n"
                 "      bw: Set bandwidth to <arg> kHz (250 or 500).\n"
                 "      len: Transmit packages with <arg> bytes payload.\n"
                 "      mode: Set to 0 for periodic transmissions, 1 for continuous wave.\n"
                 "      int: For periodic transmissions, wait <arg> ms between packages.\n"
                 "           Set <arg> to 0 to disable transmission.\n"
                 "      pwr: Set Transmit at <arg> dBm (-3 to 22) output power.\n"
                 "  Example: set bandwidth to 250 kHz\n"
                 "      lora bw 250\n"
                 "  Arguments:\n",

        .hint = NULL,
        .func = &processLoraCommand,
        .argtable = &lora_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}

static void registerLoraHFCommand()
{
    lora_args.setting =
        arg_strn(NULL, NULL, "<setting>", 0, 1,
                 "Change the LoRaHF setting named <setting>."
                 );
    lora_args.arg =
        arg_intn(NULL, NULL, "<arg>", 0, 1,
                 "Integer argument the settings.");
    lora_args.end = arg_end(10);

    const esp_console_cmd_t cmd = {
        .command = "lorahf",
        .help = "Change SX1280 LoRaHF Radio Settings.\n"
                 "  Settings accept one integer argument <arg>.\n"
                 "  Supported settings are:\n"
                 "      freq: Transmit at RF frequency <arg> Hz.\n"
                 "      bw: Set bandwidth to <arg> kHz (800 or 1600).\n"
                 "      len: Transmit packages with <arg> bytes payload.\n"
                 "      mode: Set to 0 for periodic transmissions, 1 for continuous wave.\n"
                 "      int: For periodic transmissions, wait <arg> ms between packages.\n"
                 "           Set <arg> to 0 to disable transmission.\n"
                 "      pwr: Set Transmit at <arg> dBm (-18 to 13) output power.\n"
                 "  Example: set bandwidth to 1600 kHz\n"
                 "      lorahf bw 1600\n"
                 "  Arguments:\n",
        .hint = NULL,
        .func = &processLoraHFCommand,
        .argtable = &lora_args
    };
    ESP_ERROR_CHECK( esp_console_cmd_register(&cmd) );
}


//
// UART configuration for the command line
//

void cmdLineInitUart()
{
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Configure UART. Note that REF_TICK is used so that the baud rate remains
     * correct while APB frequency is changing in light sleep mode.
     */
    uart_config_t uart_config = {};
    uart_config.baud_rate = CONFIG_CONSOLE_UART_BAUDRATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.use_ref_tick = true;

    ESP_ERROR_CHECK( uart_param_config(CMD_LINE_UART_NUMBER, &uart_config) );

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(CMD_LINE_UART_NUMBER, 256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(CMD_LINE_UART_NUMBER);

    /* Initialize the console */
    esp_console_config_t console_config = {};
    console_config.max_cmdline_args = 8;
    console_config.max_cmdline_length = 256;
    
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);
} // end inituart


void cmdLineInit()
{
    // configure uart
    cmdLineInitUart();

    // register commands
    esp_console_register_help_command();
    registerVersion();
    registerLoraCommand();
    registerLoraHFCommand();
} // end init


void loraInit()
{
    // chip select pins
    pinMode(LORA_CS, OUTPUT);
    pinMode(LORAHF_CS, OUTPUT);
    pinMode(DISPLAY_CS, OUTPUT);

    // chip select LoRa
    digitalWrite(LORA_CS, LOW);
    digitalWrite(LORAHF_CS, HIGH);
    digitalWrite(DISPLAY_CS, HIGH);

    lora.RadioNss = LORA_CS;
    lora.RadioSpi = &SPI;

    pinMode(SS, OUTPUT);
    pinMode(LORA_TXEN,OUTPUT);
    pinMode(LORA_RXEN,OUTPUT);
    digitalWrite(LORA_TXEN,LOW);
    digitalWrite(LORA_RXEN,HIGH);

    // start lora module
    lora.Init();
    lora.SetStandby( STDBY_XOSC );
    lora.ClearIrqStatus( IRQ_RADIO_ALL );

    ModulationParams_t ModulationParams;
    ModulationParams.PacketType = PACKET_TYPE_LORA;
    ModulationParams.Params.LoRa.SpreadingFactor = LORA_SF7;
    ModulationParams.Params.LoRa.Bandwidth       = LORA_BW_125;
    ModulationParams.Params.LoRa.CodingRate      = LORA_CR_4_5;
    lora.SetPacketType( ModulationParams.PacketType );
    lora.SetModulationParams( &ModulationParams );

    PacketParams.PacketType     = PACKET_TYPE_LORA;
    PacketParams.Params.LoRa.PreambleLength      =   8;                            
    PacketParams.Params.LoRa.HeaderType          = LORA_PACKET_EXPLICIT;
    PacketParams.Params.LoRa.PayloadLength       = 12;
    PacketParams.Params.LoRa.CrcMode             = LORA_CRC_ON;
    PacketParams.Params.LoRa.InvertIQ            = LORA_IQ_NORMAL;
    lora.SetPacketParams( &PacketParams );
  
    lora.SetRfFrequency(915000000 );
    lora.SetBufferBaseAddresses( 0x00, 0x00 );

    lora.SetTxParams(22, RADIO_RAMP_200_US );
    lora.SetRxTxFallbackMode(0x40);  //fallback to FS
    //LoRa.SetTxContinuousWave();
    //while(1);
    
    lora.SetRegulatorMode(USE_DCDC);
    digitalWrite(LORA_TXEN,LOW);
    digitalWrite(LORA_RXEN,HIGH);
    lora.SetDioIrqParams( IRQ_RX_DONE|IRQ_TX_DONE|IRQ_PREAMBLE_DETECTED|IRQ_CRC_ERROR, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE );

    // power up, nss low is active chip select
    lora.RadioNssSet(0);
    lora.RadioNssSet(1);
    lora.RadioNssSet(0);  
    delay(1);
    lora.SetStandby(STDBY_RC);
    digitalWrite(LORA_TXEN,LOW);
    digitalWrite(LORA_RXEN,HIGH);
    delay(100);
    printf("lora setup done, chip mode %d.\n", lora.GetStatus().Fields.ChipMode);
} // end lorainit

void loraSend()
{
    int payload_length = 8;

    digitalWrite(LORA_TXEN, HIGH);
    digitalWrite(LORA_RXEN, LOW);

    PacketParams.Params.LoRa.PayloadLength = payload_length;
    lora.SetPacketParams( &PacketParams );

    lora.SetPayload(send_buffer, payload_length);
    lora.SetTx(0);

    uint16_t iStat = lora.GetIrqStatus();
    RadioStatus_t rStat = lora.GetStatus();
    int timeout = 100;
    while ((iStat&IRQ_TX_DONE)==0 && (timeout>0))
    {
        delay(1);
        iStat = lora.GetIrqStatus();
        timeout--;
        printf("wait to tx %x  %x\n",rStat.Fields.CmdStatus,iStat);
    }
    if (timeout==0)
    {
        printf("Timeout sending\n");
    }

    digitalWrite(LORA_TXEN, LOW);
    digitalWrite(LORA_RXEN, HIGH);
    lora.SetRx(0xffffff);
} // end lorasend