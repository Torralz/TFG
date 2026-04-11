idf_component_register(
    SRCS "pruebaWroom.c" "nimble.c" "matrix/matrix.c" "matrix/font_6x10.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        bt
        driver
        esp_adc
)
