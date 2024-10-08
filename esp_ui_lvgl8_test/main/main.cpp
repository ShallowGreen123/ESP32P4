#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
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

/* These are built-in app examples in `esp-ui` library */
#include "app_examples/phone/simple_conf/src/phone_app_simple_conf.hpp"
#include "app_examples/phone/complex_conf/src/phone_app_complex_conf.hpp"
#include "app_examples/phone/squareline/src/phone_app_squareline.hpp"

static const char *TAG = "main";

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(bsp_spiffs_mount());
    ESP_LOGI(TAG, "SPIFFS mount successfully");

    ESP_ERROR_CHECK(bsp_extra_codec_init());

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

    ESP_UI_StyleSize_t calibrate_size = (*phone_stylesheet).core.screen_size;
    ESP_UI_LOGI("Add stylesheet(%s - %dx%d)", (*phone_stylesheet).core.name, calibrate_size.width, calibrate_size.height);

    ESP_UI_CHECK_FALSE_EXIT(phone->addStylesheet(*phone_stylesheet), "Add phone stylesheet failed");
    ESP_UI_CHECK_FALSE_EXIT(phone->activateStylesheet(*phone_stylesheet), "Activate phone stylesheet failed");

    assert(phone->begin() && "Failed to begin phone");

    PhoneAppSquareline *smart_gadget = new PhoneAppSquareline(true, true);
    assert(smart_gadget != nullptr && "Failed to create phone app squareline");
    assert((phone->installApp(smart_gadget) >= 0) && "Failed to install phone app squareline");

    Calculator *calculator = new Calculator();
    assert(calculator != nullptr && "Failed to create calculator");
    assert((phone->installApp(calculator) >= 0) && "Failed to begin calculator");

    Game2048 *game_2048 = new Game2048();
    assert(game_2048 != nullptr && "Failed to create game_2048");
    assert((phone->installApp(game_2048) >= 0) && "Failed to begin game_2048");

    // AppSettings *app_settings = new AppSettings();
    // assert(app_settings != nullptr && "Failed to create app_settings");
    // assert((phone->installApp(app_settings) >= 0) && "Failed to begin app_settings");

    /* Install apps */
    PhoneAppSimpleConf *phone_app_simple_conf = new PhoneAppSimpleConf(true, true);
    ESP_UI_CHECK_NULL_EXIT(phone_app_simple_conf, "Create phone app simple conf failed");
    ESP_UI_CHECK_FALSE_EXIT((phone->installApp(phone_app_simple_conf) >= 0), "Install phone app simple conf failed");
    PhoneAppComplexConf *phone_app_complex_conf = new PhoneAppComplexConf(true, true);
    ESP_UI_CHECK_NULL_EXIT(phone_app_complex_conf, "Create phone app complex conf failed");
    ESP_UI_CHECK_FALSE_EXIT((phone->installApp(phone_app_complex_conf) >= 0), "Install phone app complex conf failed");

    // PhoneAppComplexConf *phone_app_complex_conf1 = new PhoneAppComplexConf(false, true);
    // ESP_UI_CHECK_NULL_EXIT(phone_app_complex_conf1, "Create phone app complex conf failed");
    // ESP_UI_CHECK_FALSE_EXIT((phone->installApp(phone_app_complex_conf1) >= 0), "Install phone app complex conf failed");

    // PhoneAppComplexConf *phone_app_complex_conf2 = new PhoneAppComplexConf(true, false);
    // ESP_UI_CHECK_NULL_EXIT(phone_app_complex_conf2, "Create phone app complex conf failed");
    // ESP_UI_CHECK_FALSE_EXIT((phone->installApp(phone_app_complex_conf2) >= 0), "Install phone app complex conf failed");

    // PhoneAppComplexConf *phone_app_complex_conf3 = new PhoneAppComplexConf(false, false);
    // ESP_UI_CHECK_NULL_EXIT(phone_app_complex_conf3, "Create phone app complex conf failed");
    // ESP_UI_CHECK_FALSE_EXIT((phone->installApp(phone_app_complex_conf3) >= 0), "Install phone app complex conf failed");


    bsp_display_unlock();
}