#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_amoled_rm69a10.h"
#include "esp_ldo_regulator.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lv_demos.h"

#include "bsp_display.h"

static lv_indev_t *disp_indev = NULL;


void app_main(void)
{
    bsp_display_config_t disp_config = {
        .dpi_fb_buf_num = 1,
    };
    lv_disp_t *disp = NULL;

    esp_lcd_panel_handle_t lcd = NULL;           // LCD panel handle
    // esp_lcd_touch_handle_t tp = NULL;            // LCD touch panel handle

    ESP_ERROR_CHECK(bsp_display_new(&disp_config, &lcd, NULL));
    // bsp_touch_new(NULL, &tp);
    // ESP_ERROR_CHECK(bsp_lvgl_port_init(lcd, tp, &disp, &disp_indev));
    // bsp_display_brightness_init();

    if (example_lvgl_lock(-1)) {
        // example_lvgl_demo_ui(display);

        // lv_demo_music();
        lv_demo_benchmark();
        // lv_demo_widgets();
        // lv_demo_stress();

        // Release the mutex
        example_lvgl_unlock();
    }
}