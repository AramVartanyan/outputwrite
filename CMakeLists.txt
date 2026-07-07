idf_component_register(SRCS "outputwrite.c" "led_indicator.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_adc freertos)
