idf_component_register(
    SRCS "esp32p4_board.c" "bsp_board_extra.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver
    PRIV_REQUIRES esp_lcd usb spiffs fatfs
)
