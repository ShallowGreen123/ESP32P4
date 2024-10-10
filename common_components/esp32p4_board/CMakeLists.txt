idf_component_register(
    SRCS "bsp_board_extra.c" "esp32p4_board.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver
    PRIV_REQUIRES esp_lcd usb spiffs fatfs

    REQUIRES driver
    PRIV_REQUIRES esp_timer fatfs esp_psram esp_mm
)
