idf_component_register(
    SRCS "app_main.c" "esp_foc.c" "esp_svpwm.c" "foc_control.c" "as5600.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)