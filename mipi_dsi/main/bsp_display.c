#include "bsp_display.h"
#include "esp_ldo_regulator.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "sdkconfig.h"
#include "esp_amoled_rm69a10.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "lvgl.h"
#include "lv_demos.h"

static const char *TAG = "bsp_display";

//
#if CONFIG_LV_COLOR_DEPTH == 16
#define  MIPI_DPI_PX_FORMAT         (LCD_COLOR_PIXEL_FORMAT_RGB565)
#define BSP_LCD_COLOR_DEPTH         (16)
#elif CONFIG_LV_COLOR_DEPTH >= 24
#define  MIPI_DPI_PX_FORMAT         (LCD_COLOR_PIXEL_FORMAT_RGB888)
#define BSP_LCD_COLOR_DEPTH         (24)
#endif


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD Spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FPS = 80000000/(40+140+40+800)/(4+16+16+1280) = 60Hz
#define BSP_MIPI_DSI_DPI_CLK_MHZ  80
#define BSP_MIPI_DSI_LCD_H_RES    568
#define BSP_MIPI_DSI_LCD_V_RES    1232
#define BSP_MIPI_DSI_LCD_HSYNC    40
#define BSP_MIPI_DSI_LCD_HBP      140
#define BSP_MIPI_DSI_LCD_HFP      40
#define BSP_MIPI_DSI_LCD_VSYNC    4
#define BSP_MIPI_DSI_LCD_VBP      16
#define BSP_MIPI_DSI_LCD_VFP      16


#define BSP_MIPI_DSI_LANE_NUM          2    // 2 data lanes
#define BSP_MIPI_DSI_LANE_BITRATE_MBPS 1000 // 1Gbps

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your Board Design //////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define BSP_LDO_MIPI_CHAN               3
#define BSP_LDO_MIPI_VOLTAGE_MV         2500
#define BSP_LCD_BK_LIGHT_ON_LEVEL       1
#define BSP_LCD_BK_LIGHT_OFF_LEVEL      !BSP_LCD_BK_LIGHT_ON_LEVEL
#define BSP_PIN_NUM_BK_LIGHT            26
#define BSP_PIN_NUM_LCD_RST             27

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your Application ///////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LVGL_DRAW_BUF_LINES    200
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

static SemaphoreHandle_t lvgl_api_mux = NULL;

/////////////////////////////////////////////////////// 
static bsp_display_on_trans_done_cb_t trans_done_cb;
static bsp_display_on_vsync_cb_t vsync_cb;
static esp_lcd_dsi_bus_handle_t mipi_dsi_bus = NULL;
static esp_lcd_panel_io_handle_t mipi_dbi_io = NULL;
static esp_lcd_panel_handle_t mipi_dpi_panel = NULL;
static esp_lcd_panel_handle_t rm69a10_ctrl_panel= NULL;

/////////////////////////////////////////////////////// 

// LVGL
static lv_disp_drv_t disp_drv;

 bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_api_mux, timeout_ticks) == pdTRUE;
}

 void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_api_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

    /* Just copy data from the color map to the RGB frame buffer */
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    // lv_disp_flush_ready(&disp_drv);
}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds have elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}


void lvgl_port_init(void)
{
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    static lv_disp_draw_buf_t disp_buf = { 0 }; 

    void *buf1 = NULL;
    void *buf2 = NULL;
    size_t buffer_size = 0;

    buffer_size = BSP_MIPI_DSI_LCD_H_RES * EXAMPLE_LVGL_DRAW_BUF_LINES * sizeof(lv_color_t);
    buf1 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    assert(buf1);
    buf2 = heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    assert(buf2);
    ESP_LOGI(TAG, "LVGL buffer size: %dKB", buffer_size * LV_COLOR_DEPTH / 8 / 1024);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, buffer_size);

    ESP_LOGD(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = BSP_MIPI_DSI_LCD_H_RES;
    disp_drv.ver_res = BSP_MIPI_DSI_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = mipi_dpi_panel;

    lv_disp_drv_register(&disp_drv);


    ESP_LOGI(TAG, "Use esp_timer as LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    // LVGL APIs are meant to be called across the threads without protection, so we use a mutex here
    lvgl_api_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_api_mux);

    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    
}


// MIPI AMLOED
void bsp_ldo_power_on(void)
{
    static bool is_ldo_powered = false;

    if (is_ldo_powered) {
        return;
    }
    is_ldo_powered = true;

    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = BSP_LDO_MIPI_CHAN,
        .voltage_mv = BSP_LDO_MIPI_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
}

IRAM_ATTR static bool on_color_trans_done(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    // BaseType_t need_yield = pdFALSE;
    // if (trans_done_cb) {
    //     if (trans_done_cb(panel)) {
    //         need_yield = pdTRUE;
    //     }
    // }
    // return (need_yield == pdTRUE);

    lv_disp_flush_ready(&disp_drv);
    return true;
    
}

IRAM_ATTR static bool on_vsync(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    BaseType_t need_yield = pdFALSE;
    if (vsync_cb) {
        if (vsync_cb(panel)) {
            need_yield = pdTRUE;
        }
    }

    return (need_yield == pdTRUE);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    bsp_ldo_power_on();

    ESP_LOGI(TAG, "Install LCD driver");
    esp_lcd_dsi_bus_config_t bus_config =    {
        .bus_id = 0,
        .num_data_lanes = BSP_MIPI_DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = BSP_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,   // according to the LCD xxx spec
        .lcd_param_bits = 8, // according to the LCD xxx spec
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io));

    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .bits_per_pixel = BSP_LCD_COLOR_DEPTH,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .reset_gpio_num = BSP_PIN_NUM_LCD_RST,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_rm69a10(mipi_dbi_io, &lcd_dev_config, &rm69a10_ctrl_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(rm69a10_ctrl_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(rm69a10_ctrl_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(rm69a10_ctrl_panel, true));
    
    ESP_LOGI(TAG, "Install MIPI DSI LCD data panel");
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = BSP_MIPI_DSI_DPI_CLK_MHZ,
        .pixel_format = MIPI_DPI_PX_FORMAT,
        .video_timing = {
            .h_size = BSP_MIPI_DSI_LCD_H_RES,
            .v_size = BSP_MIPI_DSI_LCD_V_RES,
            .hsync_back_porch =  BSP_MIPI_DSI_LCD_HBP,
            .hsync_pulse_width = BSP_MIPI_DSI_LCD_HSYNC,
            .hsync_front_porch = BSP_MIPI_DSI_LCD_HFP,
            .vsync_back_porch =  BSP_MIPI_DSI_LCD_VBP,
            .vsync_pulse_width = BSP_MIPI_DSI_LCD_VSYNC,
            .vsync_front_porch = BSP_MIPI_DSI_LCD_VFP,
        },
#if CONFIG_BSP_LCD_DSI_USE_DMA2D
        .flags.use_dma2d = true,
#endif
    };

    if (config != NULL) {
        dpi_config.num_fbs = config->dpi_fb_buf_num;
    }
    ESP_ERROR_CHECK(esp_lcd_new_panel_dpi(mipi_dsi_bus, &dpi_config, &mipi_dpi_panel));

    gpio_set_level(BSP_PIN_NUM_LCD_RST, 1);
    // register event callbacks
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_color_trans_done = on_color_trans_done,
        .on_refresh_done = on_vsync,
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(mipi_dpi_panel, &cbs, NULL));
    ESP_ERROR_CHECK(esp_lcd_panel_init(mipi_dpi_panel));

    if (ret_io) {
        *ret_io = mipi_dbi_io;
    }
    if (ret_panel) {
        *ret_panel = mipi_dpi_panel;
    }

    lvgl_port_init();

    return ESP_OK;
}

esp_err_t bsp_display_register_callback(bsp_display_callback_t *callback)
{
#if CONFIG_LCD_RGB_ISR_IRAM_SAFE
    if (callback) {
        ESP_RETURN_ON_FALSE(esp_ptr_in_iram(callback), ESP_ERR_INVALID_ARG, TAG, "Callback not in IRAM");
    }
#endif
    trans_done_cb = callback->on_trans_done_cb;
    vsync_cb = callback->on_vsync_cb;

    return ESP_OK;
}
