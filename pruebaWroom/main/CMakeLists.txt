idf_component_register(
    SRCS "pruebaWroom.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        bt
        driver
        esp_adc
)
