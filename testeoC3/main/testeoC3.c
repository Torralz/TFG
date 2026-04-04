#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Mapeo real para Seeed Studio XIAO ESP32-C3
// Serigrafía A0 -> GPIO 2 (ADC1_CH2)
// Serigrafía A1 -> GPIO 3 (ADC1_CH3)
#define XIAO_A0_GPIO    2
#define XIAO_A1_GPIO    3
#define XIAO_A0_CH      ADC_CHANNEL_2
#define XIAO_A1_CH      ADC_CHANNEL_3

// Pines para botones (ajustados según tu comentario anterior de que 4,5,6 funcionan)
#define BTN_1_GPIO      4   // D2 / A2 en XIAO
#define BTN_2_GPIO      5   // D3 / A3 en XIAO
#define BTN_3_GPIO      6   // D8 en XIAO (SDA)

static const char *TAG = "XIAO_DIAG";

void app_main(void) {
    ESP_LOGI(TAG, "DIAGNÓSTICO ESPECIAL PARA XIAO ESP32-C3");

    // 1. Configuración del ADC
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11, // Necesario para rango 0-3.3V en el XIAO
    };
    
    // Configuramos los canales 2 y 3 (Pines A0 y A1)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, XIAO_A0_CH, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, XIAO_A1_CH, &chan_cfg));

    // 2. Configuración de Botones
    uint64_t pin_mask = (1ULL << BTN_1_GPIO) | (1ULL << BTN_2_GPIO) | (1ULL << BTN_3_GPIO);
    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Iniciado. Lectura de pines físicos A0 y A1 del XIAO.");

    int val_a0, val_a1;
    while (1) {
        // Leer A0 y A1
        adc_oneshot_read(adc1_handle, XIAO_A0_CH, &val_a0);
        adc_oneshot_read(adc1_handle, XIAO_A1_CH, &val_a1);

        // Leer botones
        int b4 = gpio_get_level(BTN_1_GPIO);
        int b5 = gpio_get_level(BTN_2_GPIO);
        int b6 = gpio_get_level(BTN_3_GPIO);

        printf("XIAO A0(G2): %4d | XIAO A1(G3): %4d || BTNS -> D2:%d D3:%d D8:%d\n",
               val_a0, val_a1, b4, b5, b6);

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}
