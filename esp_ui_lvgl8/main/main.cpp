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

// esp32p4_board
#include "esp-bsp.h"
#include "display.h"

// esp-ui
#include "esp_ui.hpp"
#include "esp_ui_phone_568_1232_stylesheet.h"
#include "app_examples/phone/squareline/src/phone_app_squareline.hpp"
#include "apps.h"

extern "C" void app_main(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * BSP_LCD_V_RES,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .sw_rotate = false,
        }};
    lv_disp_t *disp = bsp_display_start_with_config(&cfg);

    // bsp_display_backlight_on();  // AMOLED no backlight

    bsp_display_lock(0);

    /* LVGL demo */
    // lv_demo_benchmark();
    // lv_demo_music();
    // lv_demo_stress();

    /* esp-ui demo */
    ESP_UI_Phone *phone = new ESP_UI_Phone(disp);
    assert(phone != nullptr && "Failed to create phone");

    ESP_UI_PhoneStylesheet_t *phone_stylesheet = new ESP_UI_PhoneStylesheet_t ESP_UI_PHONE_568_1232_DARK_STYLESHEET();
    ESP_UI_CHECK_NULL_EXIT(phone_stylesheet, "Create phone stylesheet failed");
    ESP_UI_CHECK_FALSE_EXIT(phone->addStylesheet(*phone_stylesheet), "Add phone stylesheet failed");
    ESP_UI_CHECK_FALSE_EXIT(phone->activateStylesheet(*phone_stylesheet), "Activate phone stylesheet failed");

    assert(phone->begin() && "Failed to begin phone");

    PhoneAppSquareline *smart_gadget = new PhoneAppSquareline(true, true);
    assert(smart_gadget != nullptr && "Failed to create phone app squareline");
    assert((phone->installApp(smart_gadget) >= 0) && "Failed to install phone app squareline");

    Calculator *calculator = new Calculator();
    assert(calculator != nullptr && "Failed to create calculator");
    assert((phone->installApp(calculator) >= 0) && "Failed to begin calculator");

    bsp_display_unlock();
}