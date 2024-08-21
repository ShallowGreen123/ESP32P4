
#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define BSP_I2C_NUM 0
#define BSP_I2C_SCL 8
#define BSP_I2C_SDA 7

#define BSP_LCD_H_RES (568)
#define BSP_LCD_V_RES (1232)
#define MIPI_DSI_LANE_MBPS (1000)

#define BSP_TOUCH_MIRROR_X (true)
#define BSP_TOUCH_MIRROR_Y (true)

// static const char *TAG = "P4-FUNCTION-EV-BOARD";

static bool i2c_initialized = false;
static i2c_master_bus_handle_t i2c_bus_handle;

esp_err_t bsp_i2c_init(void)
{
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

esp_err_t bsp_i2c_deinit(void)
{
    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus_handle));
    return ESP_OK;
}

esp_err_t bsp_get_i2c_bus_handle(i2c_master_bus_handle_t *handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    *handle = i2c_bus_handle;
    return ESP_OK;
}

// esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
// {
//     bsp_i2c_init();

//     /* Initialize touch */
//     esp_lcd_touch_config_t tp_cfg = {
//         .x_max = BSP_LCD_H_RES,
//         .y_max = BSP_LCD_V_RES,
//         .rst_gpio_num = GPIO_NUM_NC,
//         .int_gpio_num = GPIO_NUM_NC,
//         .levels = {
//             .reset = 0,
//             .interrupt = 0,
//         },
//         .flags = {
//             .swap_xy = 0,
//             .mirror_x = BSP_TOUCH_MIRROR_X,
//             .mirror_y = BSP_TOUCH_MIRROR_Y,
//         },
//     };

//     esp_lcd_panel_io_handle_t tp_io_handle = NULL;
//     esp_lcd_panel_io_i2c_config_t tp_io_config = {
//         .dev_addr = (0xBB),
//         .control_phase_bytes = 1,
//         .dc_bit_offset = 0,
//         .lcd_cmd_bits = 16,
//         .flags = {
//             .disable_control_phase = 1,
//         }};
    
// }