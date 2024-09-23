#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "lv_examples.h"

#include "esp-bsp.h"
#include "display.h"

static const char *TAG = "MAIN";
static lv_indev_t *disp_indev = NULL;

lv_indev_t *esp_lvgl_get_input_dev(const lv_disp_t *display, lv_indev_type_t type)
{
    lv_indev_t *indev = NULL;
    lv_indev_t *indev_tmp = lv_indev_get_next(NULL);

    while (indev_tmp != NULL) {
        if (indev_tmp->driver->disp == display && indev_tmp->driver->type == type) {
            indev = indev_tmp;
            break;
        }
        indev_tmp = lv_indev_get_next(indev_tmp);
    }

    return indev;
}

static void lvgl_port_touchpad_read(lv_indev_t *indev_drv, lv_indev_data_t *data)
{
    static int32_t last_x = 0;
    static int32_t last_y = 0;

    if (bsp_touchpad_read_point(&last_x, &last_y, 1) > 0)
    {

        ESP_LOGI(TAG, "touch x=%d, y=%d", data->point.x, data->point.y);
        data->state = LV_INDEV_STATE_PR;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }

    data->point.x = lv_map(last_x, 0, 1024, 0, BSP_LCD_H_RES);
    data->point.y = lv_map(last_y, 0, 2400, 0, BSP_LCD_V_RES);
}


void app_main(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * BSP_LCD_V_RES,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .sw_rotate = false,
        }
    };
    bsp_display_start_with_config(&cfg);

    // bsp_display_backlight_on();  // AMOLED no backlight

    disp_indev = esp_lvgl_get_input_dev(lv_disp_get_default(), LV_INDEV_TYPE_POINTER);
    disp_indev->driver->read_cb = lvgl_port_touchpad_read;

    bsp_display_lock(0);

    lv_demo_benchmark();

    bsp_display_unlock();
}