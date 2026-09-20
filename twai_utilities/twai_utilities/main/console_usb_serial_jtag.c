/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_console.h"
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "console_usb_serial_jtag.h"

static const char *TAG = "usb_console";

#define USB_CONSOLE_LINE_MAX_LEN   256
#define USB_CONSOLE_TASK_STACK     4096
#define USB_CONSOLE_TASK_PRIORITY  5

static const char s_prompt[] = "twai>";

static inline void usb_write(const void *data, size_t len)
{
    usb_serial_jtag_write_bytes(data, len, portMAX_DELAY);
}

static inline void usb_write_str(const char *s)
{
    usb_write(s, strlen(s));
}

static void console_usb_serial_jtag_task(void *arg)
{
    char line[USB_CONSOLE_LINE_MAX_LEN];
    size_t len = 0;
    uint8_t ch;

    usb_write_str("\r\n");
    usb_write_str(s_prompt);

    while (1) {
        int read = usb_serial_jtag_read_bytes(&ch, 1, portMAX_DELAY);
        if (read <= 0) {
            continue;
        }

        if (ch == '\r' || ch == '\n') {
            usb_write_str("\r\n");
            line[len] = '\0';

            if (len > 0) {
                int cmd_ret = 0;
                esp_err_t err = esp_console_run(line, &cmd_ret);
                if (err == ESP_ERR_NOT_FOUND) {
                    usb_write_str("Unrecognized command\r\n");
                } else if (err == ESP_ERR_INVALID_ARG) {
                    /* Empty command line, nothing to run */
                } else if (err != ESP_OK) {
                    ESP_LOGE(TAG, "esp_console_run failed: 0x%x", err);
                }
            }

            len = 0;
            usb_write_str(s_prompt);
            continue;
        }

        if (ch == 0x7f || ch == 0x08) {
            /* Backspace / delete */
            if (len > 0) {
                len--;
                usb_write_str("\b \b");
            }
            continue;
        }

        if (ch < 0x20) {
            /* Ignore other control characters */
            continue;
        }

        if (len < USB_CONSOLE_LINE_MAX_LEN - 1) {
            line[len++] = (char)ch;
            usb_write(&ch, 1);
        }
    }
}

void console_usb_serial_jtag_start(void)
{
    if (!usb_serial_jtag_is_driver_installed()) {
        usb_serial_jtag_driver_config_t usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
        esp_err_t err = usb_serial_jtag_driver_install(&usb_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install USB Serial/JTAG driver: 0x%x", err);
            return;
        }
    }

    /* Route the secondary console's log/printf mirroring through the driver
     * too, so it shares the same thread-safe TX/RX path as this task instead
     * of racing with the default raw register access used for log output. */
    usb_serial_jtag_vfs_use_driver();

    xTaskCreate(console_usb_serial_jtag_task, "usb_console", USB_CONSOLE_TASK_STACK,
                NULL, USB_CONSOLE_TASK_PRIORITY, NULL);
}
