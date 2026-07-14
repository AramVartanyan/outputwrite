# Per-target component dependencies — see README.md, "ESP8266_RTOS_SDK notes".
# IDF_TARGET is a build property, not a plain variable, so it must be read
# via idf_build_get_property (a bare ${IDF_TARGET} is empty here).
idf_build_get_property(_ow_target IDF_TARGET)
if(_ow_target STREQUAL "esp8266")
    idf_component_register(SRCS "outputwrite.c" "led_indicator.c"
                        INCLUDE_DIRS "."
                        REQUIRES esp8266 freertos)
else()
    idf_component_register(SRCS "outputwrite.c" "led_indicator.c"
                        INCLUDE_DIRS "."
                        REQUIRES driver esp_adc freertos)
endif()
