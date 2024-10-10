/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "lvgl.h"
#include "widgets/navigation_bar/esp_brookesia_navigation_bar_type.h"

#ifdef __cplusplus
extern "C" {
#endif

LV_IMG_DECLARE(esp_brookesia_phone_568_1232_navigation_bar_image_back);
LV_IMG_DECLARE(esp_brookesia_phone_568_1232_navigation_bar_image_home);
LV_IMG_DECLARE(esp_brookesia_phone_568_1232_navigation_bar_image_recents_screen);

#define ESP_BROOKESIA_PHONE_568_1232_DARK_NAVIGATION_BAR_DATA()                                                      \
    {                                                                                                        \
        .main = {                                                                                            \
            .size = ESP_BROOKESIA_STYLE_SIZE_RECT_W_PERCENT(100, 64),                                               \
            .background_color = ESP_BROOKESIA_STYLE_COLOR(0x38393A),                                                \
        },                                                                                                   \
        .button = {                                                                                          \
            .icon_size = ESP_BROOKESIA_STYLE_SIZE_SQUARE(32),                                                       \
            .icon_images = {                                                                                 \
                ESP_BROOKESIA_STYLE_IMAGE_RECOLOR_WHITE(&esp_brookesia_phone_568_1232_navigation_bar_image_back),           \
                ESP_BROOKESIA_STYLE_IMAGE_RECOLOR_WHITE(&esp_brookesia_phone_568_1232_navigation_bar_image_home),           \
                ESP_BROOKESIA_STYLE_IMAGE_RECOLOR_WHITE(&esp_brookesia_phone_568_1232_navigation_bar_image_recents_screen), \
            },                                                                                               \
            .navigate_types = {                                                                              \
                ESP_BROOKESIA_CORE_NAVIGATE_TYPE_BACK,                                                              \
                ESP_BROOKESIA_CORE_NAVIGATE_TYPE_HOME,                                                              \
                ESP_BROOKESIA_CORE_NAVIGATE_TYPE_RECENTS_SCREEN,                                                    \
            },                                                                                               \
            .active_background_color = ESP_BROOKESIA_STYLE_COLOR_WITH_OPACIRY(0xFFFFFF, LV_OPA_50),                 \
        },                                                                                                   \
        .visual_flex = {                                                                             \
            .show_animation_time_ms = 200,                                                           \
            .show_animation_delay_ms = 0,                                                            \
            .show_animation_path_type = ESP_BROOKESIA_LV_ANIM_PATH_TYPE_EASE_OUT,                           \
            .show_duration_ms = 2000,                                                                \
            .hide_animation_time_ms = 200,                                                           \
            .hide_animation_delay_ms = 0,                                                            \
            .hide_animation_path_type = ESP_BROOKESIA_LV_ANIM_PATH_TYPE_EASE_IN,                            \
        },                                                                                           \
        .flags = {                                                                                   \
            .enable_main_size_min = 0,                                                               \
            .enable_main_size_max = 0,                                                               \
        },                                                                                           \
    }

#ifdef __cplusplus
}
#endif
