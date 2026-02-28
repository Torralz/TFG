#include <stdio.h>
#include <stdlib.h>     // para abs()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"

#define JOY_X_CHANNEL   ADC1_CHANNEL_6   // GPIO34
#define JOY_Y_CHANNEL   ADC1_CHANNEL_7   // GPIO35
#define JOY_SW_GPIO     32               // SW del joystick
#define BUTTON_1_GPIO   33               // Selector de capa 1
#define BUTTON_2_GPIO   25               // Selector de capa 2

int leer_joystick(void);

// Tabla de 4 capas x 8 posiciones (caracteres, no strings)
static const char diccionario[4][8] = {
    { 'e','a','o','s','r','n','i','d' },
    { 'l','c','t','u','m','p','b','g' },
    { 'v','y','q','h','f','z','j','ñ' },
    { 'x','k','w',' ',' ',' ',' ',' ' }
};

void app_main(void)
{
    // Configurar ADC1: 12 bits, atenuación 11 dB (rango aprox. 0–3.3 V)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(JOY_X_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(JOY_Y_CHANNEL, ADC_ATTEN_DB_11);

    // Configurar GPIOs de botones con pull-up interno
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOY_SW_GPIO) | (1ULL << BUTTON_1_GPIO) | (1ULL << BUTTON_2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    char ultima_letra = '@';

    while (1) {
        int x  = adc1_get_raw(JOY_X_CHANNEL);
        int y  = adc1_get_raw(JOY_Y_CHANNEL);
        int sw = gpio_get_level(JOY_SW_GPIO);     // 0 = pulsado (a GND), 1 = suelto
        int b1 = gpio_get_level(BUTTON_1_GPIO);   // 0 = pulsado (a GND), 1 = suelto
        int b2 = gpio_get_level(BUTTON_2_GPIO);   // 0 = pulsado (a GND), 1 = suelto

        // Ternario con paréntesis para que sea claro
        int capa = (b1 == 1)
                     ? ((b2 == 1) ? 0 : 1)
                     : ((b2 == 1) ? 2 : 3);

        // printf("Joystick X=%4d  Y=%4d  SW=%d  B1=%d  B2=%d\n", x, y, sw, b1, b2);

        int dir = leer_joystick();
        if (dir != 9) {
            char letra = diccionario[capa][dir];
            if(letra != ultima_letra || ultima_letra == ' '){
                printf("%c ", letra);
                printf("Joystick X=%4d  Y=%4d  SW=%d  B1=%d  B2=%d\n", x, y, sw, b1, b2);
                fflush(stdout);
                ultima_letra = letra;
            }
        }else{
            ultima_letra = '@';
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // 100 ms
    }
}

int leer_joystick(void)
{
    int x = adc1_get_raw(JOY_X_CHANNEL);  // GPIO34 ~1995 neutral
    int y = adc1_get_raw(JOY_Y_CHANNEL);  // GPIO35 ~1882 neutral

    // Zona muerta central (ajusta +- según tu ruido real)
    if (abs(x - 1995) < 300 && abs(y - 1882) < 300) {
        return 9;  // Centro (neutro, sin letra)
    }

    if (abs(x - 1995) < 300 && abs(y - 0) < 300) return 0;  // Norte
    if (abs(x - 4095) < 300 && abs(y - 0) < 300) return 1;  // NE
    if (abs(x - 4095) < 300 && abs(y - 1882) < 300) return 2;  // Este
    if (abs(x - 4095) < 300 && abs(y - 4095) < 300) return 3;  // SE
    if (abs(x - 1995) < 300 && abs(y - 4095) < 300) return 4;  // Sur
    if (abs(x - 0) < 300 && abs(y - 4095) < 300) return 5;  // SO
    if (abs(x - 0) < 300 && abs(y - 1882) < 300) return 6;  // Oeste
    if (abs(x - 0) < 300 && abs(y - 0) < 300) return 7;  // NO

    return 9;  // Por seguridad, trátalo como neutro
}
