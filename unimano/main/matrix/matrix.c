#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#include "font_6x10.h"

// Configuration
#define LED_STRIP_GPIO_PIN   10  // D0 on XIAO ESP32-C3
#define LED_STRIP_NUM_PIXELS 60
// #define BRIGHTNESS           20  // Keep this low (0-BRIGHTNESS) to save power!
// #define REFRESH_RATE         100

#define BUTTON_1_GPIO   5               
#define BUTTON_2_GPIO   6 
#define JOY_X_CHANNEL   3               //en verdad es Y

uint8_t REFRESH_RATE = 100;
uint8_t BRIGHTNESS = 20;

static adc_oneshot_unit_handle_t adc1_handle;
static const char *TAG = "XIAO_MATRIX";
led_strip_handle_t led_strip;

void init_led_strip() {
    ESP_LOGI(TAG, "Initializing LED strip on GPIO %d", LED_STRIP_GPIO_PIN);

    /* LED strip general configuration */
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN,
        .max_leds = LED_STRIP_NUM_PIXELS,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,                // Standard logic
    };

    /* RMT specific configuration */
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,           // C3 supports DMA, but not needed for 60 LEDs
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    // gpio_config_t io_conf = {
    //     .pin_bit_mask = (1ULL << BUTTON_1_GPIO) | (1ULL << BUTTON_2_GPIO),
    //     .mode         = GPIO_MODE_INPUT,
    //     .pull_up_en   = GPIO_PULLUP_ENABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //     .intr_type    = GPIO_INTR_DISABLE
    // };
    // ESP_ERROR_CHECK(gpio_config(&io_conf));
    //
    // adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    // ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));
    //
    // adc_oneshot_chan_cfg_t chan_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    // ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOY_X_CHANNEL, &chan_cfg));
}

// void text_test_task(){
//   draw_char_from_font('H', 20, 20, 20);
//   vTaskDelay(pdMS_TO_TICKS(1000)); // Descansa un poco la CPU
//   draw_char_from_font('O', 20, 20, 20);
//   vTaskDelay(pdMS_TO_TICKS(1000)); // Descansa un poco la CPU
//   draw_char_from_font('L', 20, 20, 20);
//   vTaskDelay(pdMS_TO_TICKS(1000)); // Descansa un poco la CPU
//   draw_char_from_font('A', 20, 20, 20);
//   vTaskDelay(pdMS_TO_TICKS(1000)); // Descansa un poco la CPU
// }

void leds_task(){
    while (1) {
        //tengo que primero aumentar R hasta max y luego G y despues disminuir return 
        for (int i = 0; i < BRIGHTNESS; i++){
            for (int j = 0; j < LED_STRIP_NUM_PIXELS; j++){
              led_strip_set_pixel(led_strip, j, BRIGHTNESS, i , 0);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(REFRESH_RATE));
        }
        
        for (int i = BRIGHTNESS; i > 0; i--){
            for (int j = 0; j < LED_STRIP_NUM_PIXELS; j++){
              led_strip_set_pixel(led_strip, j, i, BRIGHTNESS , 0);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(REFRESH_RATE));
        }
        
        for (int i = 0; i < BRIGHTNESS; i++){
            for (int j = 0; j < LED_STRIP_NUM_PIXELS; j++){
              led_strip_set_pixel(led_strip, j, 0, BRIGHTNESS , i);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(REFRESH_RATE));
        }
        
        for (int i = BRIGHTNESS; i > 0; i--){
            for (int j = 0; j < LED_STRIP_NUM_PIXELS; j++){
              led_strip_set_pixel(led_strip, j, 0, i, BRIGHTNESS);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(REFRESH_RATE));
        }
      
        for (int i = 0; i < BRIGHTNESS; i++){
            for (int j = 0; j < LED_STRIP_NUM_PIXELS; j++){
              led_strip_set_pixel(led_strip, j, i, 0 , BRIGHTNESS);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(REFRESH_RATE));
        }
        
        for (int i = BRIGHTNESS; i > 0; i--){
            for (int j = 0; j < LED_STRIP_NUM_PIXELS; j++){
              led_strip_set_pixel(led_strip, j, BRIGHTNESS, 0 , i);
            }
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(REFRESH_RATE));
        }
    }
}

void button_task(){
    while (1) {
        if (gpio_get_level(BUTTON_1_GPIO) == 1 && gpio_get_level(BUTTON_2_GPIO) == 0) {
            // Asegúrate de que no baje de 0
            if (REFRESH_RATE >= 50) { 
                REFRESH_RATE -= 50;
                ESP_LOGI(TAG, "Decreasing REFRESH_RATE by 50 currently %u", REFRESH_RATE);
            }
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }
        
        if (gpio_get_level(BUTTON_1_GPIO) == 0 && gpio_get_level(BUTTON_2_GPIO) == 1) {
            // Evita pasarte de 255 (255 - 50 = 205)
            if (REFRESH_RATE <= 205) { 
                REFRESH_RATE += 50;
                ESP_LOGI(TAG, "Increasing REFRESH_RATE by 50 currently %u", REFRESH_RATE);
            }
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
}

void brightness_task(){
    int x;
    while(1){
        if (adc_oneshot_read(adc1_handle, JOY_X_CHANNEL, &x) != ESP_OK) 
            ESP_LOGW(TAG, "Error ADC");
        BRIGHTNESS = (255 * x) / 4095;
        vTaskDelay(pdMS_TO_TICKS(50)); // Descansa un poco la CPU
  }
}

void writing_task(const char *text){
  //lo que quiero es poder escribir texto en la pantalla, como frases enteras pero esto es TODO
}


// void app_main(void) {
//     init_led_strip();
//     xTaskCreate(leds_task, "LEDs", 4096, NULL, 5, NULL);
//     xTaskCreate(button_task, "Boton", 2048, NULL, 5, NULL);
//     xTaskCreate(brightness_task, "brightness", 4096, NULL, 5, NULL);
//     //xTaskCreate(text_test_task, "LEDs", 4096, NULL, 5, NULL);
// }
