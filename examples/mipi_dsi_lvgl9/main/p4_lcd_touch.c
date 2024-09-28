
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
#include "esp_lcd_touch_gt9895.h"

#include "driver/i2c_master.h"
#include "esp_lcd_panel_io_interface.h"

#define BSP_I2C_NUM (0)
#define BSP_I2C_SCL (8)
#define BSP_I2C_SDA (7)
#define BSP_GPIO_RST (21)
#define BSP_GPIO_INT (22)

#define BSP_LCD_H_RES      (568)
#define BSP_LCD_V_RES      (1232)
// #define BSP_LCD_H_RES (1024)
// #define BSP_LCD_V_RES (600)

static const char *TAG = "TOUCH";
static i2c_master_bus_handle_t i2c_bus_handle;
static esp_lcd_panel_io_handle_t tp_io_handle = NULL;

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
        .glitch_ignore_cnt = 7,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));

    i2c_initialized = true;

    return ESP_OK;
}

void bsp_touch_new(void)
{
    printf("Initilize I2C\n");
    bsp_i2c_init();

    printf("Initilize touch\n");
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT9895_CONFIG();
    tp_io_config.scl_speed_hz = 400000;

    if (ESP_OK != i2c_master_probe(i2c_bus_handle, ESP_LCD_TOUCH_IO_I2C_GT9895_ADDRESS, 100))
    {
        ESP_LOGE(TAG, "Touch not found GT9895");
    }

    printf("Initilize touch panel\n");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle, &tp_io_config, &tp_io_handle));
}

int touchpad_read_point(int32_t *last_x, int32_t *last_y, int point_num)
{
    uint8_t buffer[90] = {0};

    esp_lcd_panel_io_rx_param(tp_io_handle, 0x10308, buffer, 90);

    uint8_t touchNum = buffer[2] & 0xF;
    int x = 0, y = 0, w = 0;
    uint8_t *ptr = &buffer[8];
    for (int i = 0; i < point_num; i++)
    {
        int id = (ptr[0] >> 4) & 0x0F;
        if (id >= 10)
        {
            break;
        }
        *last_x = *((uint16_t *)(ptr + 2));
        *last_y = *((uint16_t *)(ptr + 4));
        w = *((uint16_t *)(ptr + 6));
        ptr += 8;

        // printf("x[%d]:%d y:%d w:%d  |  ", id, x, y, w);
    }
    return (touchNum > 0);
    // if (touchNum > 0)
    //     printf("\n");
}
