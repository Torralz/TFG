idf_component_register(
    SRCS "pruebaWroom.c" "nimble.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        bt
        driver
        esp_adc
)
