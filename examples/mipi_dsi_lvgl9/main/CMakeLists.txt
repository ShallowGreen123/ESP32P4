idf_component_register(
SRCS 
    "mipi_dsi_lcd_example_main.c" 
    "lvgl_demo_ui.c"
    "p4_lcd_touch.c"
INCLUDE_DIRS 
    ".")


target_compile_options(
    ${COMPONENT_LIB}
    PRIVATE
        -Wno-format
        -Wno-incompatible-pointer-types
        -DLV_LVGL_H_INCLUDE_SIMPLE)