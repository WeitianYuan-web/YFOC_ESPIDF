idf_component_register(
    SRCS "uart_command.c" "app_main.c" "esp_foc.c" "esp_svpwm.c" "foc_control.c" "as5600.c" "motor_current_sense.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_adc
)