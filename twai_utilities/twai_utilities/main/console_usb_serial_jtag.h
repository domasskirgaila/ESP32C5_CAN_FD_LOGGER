/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start a background task that reads command lines from the USB
 *        Serial/JTAG interface and runs them through the console command
 *        registry (esp_console_run), so the console accepts commands over
 *        USB Serial/JTAG and UART at the same time.
 *
 * Commands must already be registered (register_twai_commands()) and the
 * primary REPL must already be created (esp_console_new_repl_uart /
 * esp_console_init) before calling this, since it reuses that global
 * console context instead of creating a second one.
 */
void console_usb_serial_jtag_start(void);

#ifdef __cplusplus
}
#endif
