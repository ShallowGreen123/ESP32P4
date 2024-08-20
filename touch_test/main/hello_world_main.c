/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_gt911.h"

#include "driver/i2c_master.h"

#define BSP_I2C_NUM  (0)
#define BSP_I2C_SCL  (8)
#define BSP_I2C_SDA  (7)
#define BSP_GPIO_RST (21)
#define BSP_GPIO_INT (22)

// #define BSP_LCD_H_RES      (568)
// #define BSP_LCD_V_RES      (1232)
#define BSP_LCD_H_RES      (1024)
#define BSP_LCD_V_RES      (600)

#define BSP_TOUCH_MIRROR_X (true)
#define BSP_TOUCH_MIRROR_Y (true)


#define ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG()             \
    {                                                   \
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS, \
        .control_phase_bytes = 1,                       \
        .dc_bit_offset = 0,                             \
        .lcd_cmd_bits = 16,                             \
        .flags =                                        \
        {                                               \
            .disable_control_phase = 1,                 \
        }                                               \
    }


static const char *TAG = "TOUCH";
static i2c_master_bus_handle_t i2c_bus_handle;
static esp_lcd_touch_handle_t tp = NULL;            // LCD touch panel handle

esp_err_t bsp_i2c_init(void)
{
    static bool i2c_initialized = false;
    /* I2C was initialized before */
    if (i2c_initialized)
    {
        return ESP_OK;
    }

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = BSP_I2C_NUM,
        .scl_io_num = BSP_I2C_SCL,
        .sda_io_num = BSP_I2C_SDA,
        .flags.enable_internal_pullup = false, // no pull-up
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));

    i2c_initialized = true;

    return ESP_OK;
}

static uint8_t gt911_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS;

void app_main(void)
{
    // vTaskDelay(pdMS_TO_TICKS(100));

    // esp_err_t ret = ESP_OK;
    // const gpio_config_t int_gpio_config = {
    //     .mode = GPIO_MODE_OUTPUT,
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .pull_down_en = 1,
    //     .pull_up_en = 0,
    //     .pin_bit_mask = BIT64(BSP_GPIO_INT),
    // };
    // ret = gpio_config(&int_gpio_config);

    // const gpio_config_t rst_gpio_config = {
    //     .mode = GPIO_MODE_OUTPUT,
    //     .pin_bit_mask = BIT64(BSP_GPIO_RST)
    // };
    // ret = gpio_config(&rst_gpio_config);

    // gpio_set_level(BSP_GPIO_INT, 0);
    // gpio_set_level(BSP_GPIO_RST, 0);

    // vTaskDelay(pdMS_TO_TICKS(20));

    // gpio_set_level(BSP_GPIO_RST, 1);

    // vTaskDelay(pdMS_TO_TICKS(150));

    printf("Initilize I2C\n");
    ESP_ERROR_CHECK(bsp_i2c_init());
    
    // gpio_set_level(BSP_GPIO_INT, 1);
    // vTaskDelay(pdMS_TO_TICKS(1));

    // gpio_set_level(BSP_GPIO_INT, 0);
    // vTaskDelay(pdMS_TO_TICKS(10));

    // gpio_set_level(BSP_GPIO_INT, 1);
    // vTaskDelay(pdMS_TO_TICKS(1));

    // gpio_set_level(BSP_GPIO_INT, 0);

    printf("Initilize touch\n");
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = BSP_TOUCH_MIRROR_X,
            .mirror_y = BSP_TOUCH_MIRROR_Y,
        },
        .driver_data = &gt911_addr,
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

    if(ESP_OK == i2c_master_probe(i2c_bus_handle, ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS, 100)) {
        ESP_LOGI(TAG, "Found touch GT911");
        // esp_lcd_panel_io_i2c_config_t config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
        // memcpy(&tp_io_config, &config, sizeof(config));
        tp_io_config.scl_speed_hz = 400000;
    } else {
        ESP_LOGE(TAG, "Touch not found");
    }

    printf("Initilize touch panel\n");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle, &tp_io_config, &tp_io_handle));

    printf("Initilize touch gt911\n");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));


    uint16_t touchpad_x;
    uint16_t touchpad_y;
    uint8_t touchpad_cnt = 0;
    while (1) {
        
        /* Read data from touch controller into memory */
        esp_lcd_touch_read_data(tp);

        /* Read data from touch controller */
        bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, &touchpad_x, &touchpad_y, NULL, &touchpad_cnt, 1);
        if (touchpad_pressed && touchpad_cnt > 0) {

            ESP_LOGI(TAG, "Touch position: %d,%d", touchpad_x, touchpad_y);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
