/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "soc/soc_caps.h"

#if SOC_LCD_RGB_SUPPORTED
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"

#include "../private/CheckResult.h"
#include "../bus/RGB.h"
#include "GC9503.h"

static const char *TAG = "gc9503";

/**
 * @brief LCD configuration data structure type
 *
 */
typedef struct {
    uint8_t cmd;            // LCD command
    uint8_t data[52];       // LCD data
    uint8_t data_bytes;     // Length of data in above data array; 0xFF = end of cmds.
} lcd_init_cmd_t;

// *INDENT-OFF*
const static lcd_init_cmd_t vendor_specific_init[] = {
    {0xf0, {0x55, 0xaa, 0x52, 0x08, 0x00}, 5},
    {0xf6, {0x5a, 0x87}, 2},
    {0xc1, {0x3f}, 1},
    {0xc2, {0x0e}, 1},
    {0xc6, {0xf8}, 1},
    {0xc9, {0x10}, 1},
    {0xcd, {0x25}, 1},
    {0xf8, {0x8a}, 1},
    {0xac, {0x45}, 1},
    {0xa0, {0xdd}, 1},
    {0xa7, {0x47}, 1},
    {0xfa, {0x00, 0x00, 0x00, 0x04}, 4},
    {0x86, {0x99, 0xa3, 0xa3, 0x51}, 4},
    {0xa3, {0xee}, 1},
    {0xfd, {0x3c, 0x3c, 0x00}, 3},
    {0x71, {0x48}, 1},
    {0x72, {0x48}, 1},
    {0x73, {0x00, 0x44}, 2},
    {0x97, {0xee}, 1},
    {0x83, {0x93}, 1},
    {0x9a, {0x72}, 1},
    {0x9b, {0x5a}, 1},
    {0x82, {0x2c, 0x2c}, 2},
    {0xb1, {0x10}, 1},
    {0x6d, {0x00, 0x1f, 0x19, 0x1a, 0x10, 0x0e, 0x0c, 0x0a, 0x02, 0x07, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e,
            0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x08, 0x01, 0x09, 0x0b, 0x0d, 0x0f, 0x1a, 0x19, 0x1f, 0x00}, 32},
    {0x64, {0x38, 0x05, 0x01, 0xdb, 0x03, 0x03, 0x38, 0x04, 0x01, 0xdc, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a}, 16},
    {0x65, {0x38, 0x03, 0x01, 0xdd, 0x03, 0x03, 0x38, 0x02, 0x01, 0xde, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a}, 16},
    {0x66, {0x38, 0x01, 0x01, 0xdf, 0x03, 0x03, 0x38, 0x00, 0x01, 0xe0, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a}, 16},
    {0x67, {0x30, 0x01, 0x01, 0xe1, 0x03, 0x03, 0x30, 0x02, 0x01, 0xe2, 0x03, 0x03, 0x7a, 0x7a, 0x7a, 0x7a}, 16},
    {0x68, {0x00, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a}, 13},
    {0x60, {0x38, 0x08, 0x7a, 0x7a, 0x38, 0x09, 0x7a, 0x7a}, 8},
    {0x63, {0x31, 0xe4, 0x7a, 0x7a, 0x31, 0xe5, 0x7a, 0x7a}, 8},
    {0x69, {0x04, 0x22, 0x14, 0x22, 0x14, 0x22, 0x08}, 7},
    {0x6b, {0x07}, 1},
    {0x7a, {0x08, 0x13}, 2},
    {0x7b, {0x08, 0x13}, 2},
    {0xd1, {0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47, 0x00,
            0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6, 0x02, 0xee,
            0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03,
            0xff}, 52},
    {0xd2, {0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47, 0x00,
            0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6, 0x02, 0xee,
            0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03,
            0xff}, 52},
    {0xd3, {0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47, 0x00,
            0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6, 0x02, 0xee,
            0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03,
            0xff}, 52},
    {0xd4, {0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47, 0x00,
            0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6, 0x02, 0xee,
            0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03,
            0xff}, 52},
    {0xd5, {0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47, 0x00,
            0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6, 0x02, 0xee,
            0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03,
            0xff}, 52},
    {0xd6, {0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35, 0x00, 0x47, 0x00,
            0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7, 0x02, 0x36, 0x02, 0xa6, 0x02, 0xee,
            0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5, 0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03,
            0xff}, 52},
    {0x3a, {0x66}, 1},
    {0x11, {0x00}, 0},
    {0x00, {0x00}, 0xff},
};
// *INDENT-OFF*

static esp_err_t esp_lcd_new_panel_gc9503(esp_lcd_panel_io_handle_t io_handle, const esp_lcd_rgb_panel_config_t *rgb_config, esp_lcd_panel_handle_t *ret_panel);

ESP_PanelLcd_GC9503::~ESP_PanelLcd_GC9503()
{
    if (handle) {
        del();
    }
}

void ESP_PanelLcd_GC9503::init()
{
    CHECK_NULL_RETURN(bus);
    CHECK_ERROR_RETURN(esp_lcd_new_panel_gc9503(bus->getHandle(), static_cast<ESP_PanelBus_RGB *>(bus)->getRGBConfig(), &handle));

    if (config.dev_config.reset_gpio_num >= 0) {
        gpio_config_t gpio_conf = {
            .pin_bit_mask = BIT64(config.dev_config.reset_gpio_num),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        CHECK_ERROR_RETURN(gpio_config(&gpio_conf));
    }
}

void ESP_PanelLcd_GC9503::reset()
{
    if(config.dev_config.reset_gpio_num >= 0) {
        gpio_set_level((gpio_num_t)config.dev_config.reset_gpio_num, config.dev_config.flags.reset_active_high);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)config.dev_config.reset_gpio_num, !config.dev_config.flags.reset_active_high);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    CHECK_ERROR_RETURN(esp_lcd_panel_reset(handle));
}

static esp_err_t esp_lcd_new_panel_gc9503(esp_lcd_panel_io_handle_t io_handle, const esp_lcd_rgb_panel_config_t *rgb_config, esp_lcd_panel_handle_t *ret_panel)
{
    // Initialize LCD
    // Vendor specific initialization, it can be different between manufacturers
    // Should consult the LCD supplier for initialization sequence code
    int cmd = 0;
    while (vendor_specific_init[cmd].data_bytes != 0xff) {
        esp_lcd_panel_io_tx_param(io_handle, vendor_specific_init[cmd].cmd, vendor_specific_init[cmd].data, vendor_specific_init[cmd].data_bytes);
        cmd++;
    }
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_lcd_panel_io_tx_param(io_handle, LCD_CMD_DISPON, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Create RGB panel
    ESP_RETURN_ON_ERROR(esp_lcd_new_rgb_panel(rgb_config, ret_panel), TAG, "Failed to create RGB panel");

    return ESP_OK;
}

#endif /* SOC_LCD_RGB_SUPPORTED */