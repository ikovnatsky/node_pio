#include "cmd_line.h"

#define CMD_LINE_UART_NUMBER UART_NUM_0

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

/* 'version' command */
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
        } else if ( strncmp(setting, "bw", 4) == 0 ) {
            printf("setting LoRa bandwidth to %d kHz.\n", arg);
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
        } else if ( strncmp(setting, "bw", 4) == 0 ) {
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