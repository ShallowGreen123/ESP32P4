

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
// #include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_ldo_regulator.h"
#include "esp_vfs_fat.h"
#include "usb/usb_host.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

#include "esp32p4_board.h"
#include "display.h"
#include "touch.h"
#include "bsp_err_check.h"
#include "esp_amoled_rm69a10.h"
#include "esp_lcd_touch_gt9895.h"

static const char *TAG = "BSP";

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
// static lv_indev_t *disp_indev = NULL;
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

static esp_lcd_panel_io_handle_t tp_io_handle = NULL;
static i2c_master_bus_handle_t i2c_bus_handle;

esp_err_t bsp_i2c_init(void)
{
    static bool i2c_initialized = false;
    /* I2C was initialized before */
    if (i2c_initialized)
    {
        return ESP_OK;
    }

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = BSP_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = CONFIG_BSP_I2C_CLK_SPEED_HZ
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_param_config(BSP_I2C_NUM, &i2c_conf));
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_install(BSP_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_sdcard_mount(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 64 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;
    esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return ret;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;

    const sdmmc_slot_config_t slot_config = {
        .clk = BSP_SD_CLK,
        .cmd = BSP_SD_CMD,
        .d0 = BSP_SD_D0,
        .d1 = BSP_SD_D1,
        .d2 = BSP_SD_D2,
        .d3 = BSP_SD_D3,
        .d4 = GPIO_NUM_NC,
        .d5 = GPIO_NUM_NC,
        .d6 = GPIO_NUM_NC,
        .d7 = GPIO_NUM_NC,
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 4,
        .flags = 0,
    };

    return esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

// Bit number used to represent command and parameter
#define LCD_LEDC_CH CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0};
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK};

    BSP_ERROR_CHECK_RETURN_ERR(ledc_timer_config(&LCD_backlight_timer));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&LCD_backlight_channel));
    return ESP_OK;
}

static esp_err_t bsp_enable_dsi_phy_power(void)
{
#if BSP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0
    // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
    static esp_ldo_channel_handle_t phy_pwr_chan = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = BSP_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_RETURN_ON_ERROR(esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan), TAG, "Acquire LDO channel for DPHY failed");
    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
#endif // BSP_MIPI_DSI_PHY_PWR_LDO_CHAN > 0

    return ESP_OK;
}

esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles)
{
    esp_err_t ret = ESP_OK;

    if (BSP_LCD_BACKLIGHT != GPIO_NUM_NC)
        ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");
    
    ESP_RETURN_ON_ERROR(bsp_enable_dsi_phy_power(), TAG, "DSI PHY power failed");

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = BSP_LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), TAG, "New DSI bus init failed");

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_panel_io_handle_t io;
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,   // according to the LCD RM69A10 spec
        .lcd_param_bits = 8, // according to the LCD RM69A10 spec
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io), err, TAG, "New panel IO failed");

    esp_lcd_panel_handle_t ctrl_panel = NULL;
    esp_lcd_panel_handle_t disp_panel = NULL;

    // create RM69A10 control panel
    ESP_LOGI(TAG, "Install RM69A10 LCD control panel");
    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .bits_per_pixel = 16,
        .rgb_ele_order = BSP_LCD_COLOR_SPACE,
        .reset_gpio_num = -1,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_rm69a10(io, &lcd_dev_config, &ctrl_panel), err, TAG, "New LCD panel RM69A10 failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(ctrl_panel), err, TAG, "LCD panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(ctrl_panel), err, TAG, "LCD panel init failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_disp_on_off(ctrl_panel, true), err, TAG, "LCD panel ON failed");
    esp_lcd_panel_mirror(ctrl_panel, true, true);

    ESP_LOGI(TAG, "Install MIPI DSI LCD data panel");
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = BSP_MIPI_DSI_DPI_CLK_MHZ,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .video_timing = {
            .h_size = BSP_LCD_H_RES,
            .v_size = BSP_LCD_V_RES,
            .hsync_back_porch = BSP_LCD_MIPI_DSI_LCD_HBP,
            .hsync_pulse_width = BSP_LCD_MIPI_DSI_LCD_HSYNC,
            .hsync_front_porch = BSP_LCD_MIPI_DSI_LCD_HFP,
            .vsync_back_porch = BSP_LCD_MIPI_DSI_LCD_VBP,
            .vsync_pulse_width = BSP_LCD_MIPI_DSI_LCD_VSYNC,
            .vsync_front_porch = BSP_LCD_MIPI_DSI_LCD_VFP,
        },
        .flags.use_dma2d = true,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dpi(mipi_dsi_bus, &dpi_config, &disp_panel), err, TAG, "New panel DPI failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, TAG, "New panel DPI init failed");

    /* Return all handles */
    ret_handles->io = io;
    ret_handles->mipi_dsi_bus = mipi_dsi_bus;
    ret_handles->panel = disp_panel;
    ret_handles->control = ctrl_panel;

    ESP_LOGI(TAG, "Display initialized");

    return ret;

err:
    if (disp_panel)
    {
        esp_lcd_panel_del(disp_panel);
    }
    if (ctrl_panel)
    {
        esp_lcd_panel_del(ctrl_panel);
    }
    if (io)
    {
        esp_lcd_panel_io_del(io);
    }
    if (mipi_dsi_bus)
    {
        esp_lcd_del_dsi_bus(mipi_dsi_bus);
    }
    return ret;
}

// esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
// {
//     /* Initilize I2C */
//     BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

//     /* Initialize touch */
//     const esp_lcd_touch_config_t tp_cfg = {
//         .x_max = BSP_LCD_H_RES,
//         .y_max = BSP_LCD_V_RES,
//         .rst_gpio_num = BSP_LCD_TOUCH_RST, // Shared with LCD reset
//         .int_gpio_num = BSP_LCD_TOUCH_INT,
//         .levels = {
//             .reset = 0,
//             .interrupt = 0,
//         },
//         .flags = {
//             .swap_xy = 0,
//             .mirror_x = 0,
//             .mirror_y = 0,
//         },
//     };
//     const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT9895_CONFIG();
//     ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");

//     return esp_lcd_touch_new_i2c_gt9895(tp_io_handle, &tp_cfg, ret_touch);
//     // return ESP_OK;
// }

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static void lvgl_port_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    static int32_t last_x = 0;
    static int32_t last_y = 0;

    (void)indev_drv;

    if (bsp_touchpad_read_point(&last_x, &last_y, 1) > 0)
    {
        data->point.x = lv_map(last_x, 0, 1024, 0, BSP_LCD_H_RES);
        data->point.y = lv_map(last_y, 0, 2400, 0, BSP_LCD_V_RES);

        ESP_LOGI(TAG, "touch x=%d, y=%d", data->point.x, data->point.y);
        data->state = LV_INDEV_STATE_PR;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
        }
}

static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    bsp_lcd_handles_t lcd_panels;
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new_with_handles(NULL, &lcd_panels));

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_panels.io,
        .panel_handle = lcd_panels.panel,
        .control_handle = lcd_panels.control,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = cfg->flags.buff_dma, .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
            .sw_rotate = cfg->flags.sw_rotate, /* Only SW rotation is supported for 90° and 270° */
        }};

    return lvgl_port_add_disp_dsi(&disp_cfg, NULL);
}

static lv_indev_t *bsp_display_indev_init(void)
{
    printf("Initilize I2C\n");
    ESP_ERROR_CHECK(bsp_i2c_init());

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT9895_CONFIG();
    // tp_io_config.scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ;

    // if (ESP_OK == i2c_master_probe((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM, ESP_LCD_TOUCH_IO_I2C_GT9895_ADDRESS, 100))
    // {
    //     ESP_LOGI(TAG, "Found touch GT9895");
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "Touch not found");
    // }

    printf("Initilize touch panel\n");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM, &tp_io_config, &tp_io_handle));


    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_port_touchpad_read;
    return lv_indev_drv_register(&indev_drv);
}

int bsp_touchpad_read_point(int32_t *last_x, int32_t *last_y, int point_num)
{
    uint8_t buffer[90] = {0};

    esp_lcd_panel_io_rx_param(tp_io_handle, 0x10308, buffer, 90);

    uint8_t touchNum = buffer[2] & 0xF;
    // int x = 0, y = 0, w = 0;
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
        // w = *((uint16_t *)(ptr + 6));
        ptr += 8;
        // printf("x[%d]:%d y:%d w:%d  |  ", id, x, y, w);
    }
    return (touchNum > 0);
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
            .sw_rotate = true,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    lv_display_t *disp = NULL;

    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    if (BSP_LCD_BACKLIGHT != GPIO_NUM_NC)
        bsp_display_brightness_init();

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    bsp_display_indev_init();

    return disp;
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}

#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
