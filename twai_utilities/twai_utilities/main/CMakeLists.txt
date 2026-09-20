idf_component_register(SRCS "cmd_twai_dump.c" "cmd_twai_send.c" "cmd_twai_core.c" "cmd_twai.c" "twai_utils_main.c"
                           "twai_utils_parser.c" "console_usb_serial_jtag.c"
                    REQUIRES esp_driver_twai esp_driver_gpio console esp_driver_usb_serial_jtag
                    INCLUDE_DIRS ".")
