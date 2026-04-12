#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/queue.h"

#include "nimble.h"
#include "matrix/matrix.h"
#include "matrix/font_6x10.h"

//TODO: hola

//FIXME: hola

//BUG: caca

//NOTE: notifications

//WARN: vaya: 5

//NOTE: Hecho:
//1. Desarrollar un modo de enseñar las letras letras sin escribir nada (en otro color) 
//2. Añadir funcionalidad a las teclas vacias como enter o borrar
//TODO: Quiero hacer las siguientes cosas en el futuro prox [10/4 - 12/04]
//1. Añadir Modo de escribir en mayúsculas 
//2. Añadir autocompletado (posiblemente solo en android?)
//3. Usar uno de los espacios en blanco como acceso a otra capa teclado de caracteres especiales por capas
//4. Crear una combinación de teclas que habra un menú con opciones como brillo

// GPIO / ADC Configuration
// ESP32-C3 Pins (Using valid ADC1 channels 0-4)
#define JOY_X_CHANNEL   2               // GPIO0? (Check: ADC1_CH2 is GPIO2)
#define JOY_Y_CHANNEL   3               // GPIO1? (Check: ADC1_CH3 is GPIO3)
#define JOY_SW_GPIO     4               // GPIO4
#define BUTTON_1_GPIO   5               // GPIO5
#define BUTTON_2_GPIO   6               // GPIO6

static const char *TAG = "KBD_APP";
static adc_oneshot_unit_handle_t adc1_handle;

static volatile unsigned current_layer = 0;
static volatile bool writeMode = true;
static volatile bool caps_lock_active = false; 

// HID Keycodes Dictionary (4 Layers x 8 Directions)
static const uint8_t diccionario[4][8] = {
    { 0x08, 0x04, 0x12, 0x16, 0x15, 0x11, 0x0C, 0x07 }, // e, a, o, s, r, n, i, d
    { 0x0F, 0x06, 0x18, 0x10, 0x13, 0x17, 0x05, 0x0A }, // l, c, u, m, p, t, b, g u(t), m(u), p(m), t(p), 
    { 0x19, 0x1C, 0x14, 0x0B, 0x09, 0x1D, 0x0D, 0x33 }, // v, y, q, h, f, z, j, ñ 
    { 0x1B, 0x0E, 0x1A, 0x28, 0x2A, 0x39, 0x2C, 0x2C }, // x, k, w, space...
};

static const char* diccionario_char[4][8] = {
    { "e", "a", "o", "s", "r", "n", "i", "d" }, // Verde
    { "l", "c", "u", "m", "p", "t", "b", "g" }, // Azul
    { "v", "y", "q", "h", "f", "z", "j", "ñ" }, // Rosa 
    { "x", "k", "w", "*", "<", "^", " ", " " }, // Rojo
};

static QueueHandle_t gpio_evt_queue = NULL;

// 2. The ISR: Kept as simple as possible
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)(uintptr_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// 3. The Task: Where the simultaneous logic happens
static void button_handler_task(void* arg) {
    uint32_t io_num;
    for(;;) {
        // Wait for ANY of the three switches to trigger an interrupt
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            
            // Give the user 50ms for their other fingers to press the other switches
            // This also acts as an excellent debounce delay!
            vTaskDelay(pdMS_TO_TICKS(50));

            // Read the current physical state of all three pins
            // !gpio_get_level returns 1 if pressed (assuming pull-up)
            int s1_state = !gpio_get_level(BUTTON_1_GPIO);
            int s2_state = !gpio_get_level(BUTTON_2_GPIO);
            int s3_state = !gpio_get_level(JOY_SW_GPIO);
            current_layer = (s1_state << 1) | s2_state;

            // Check if all 3 are pressed
            if (s1_state == 1 && s2_state == 1 && s3_state == 1) {
                writeMode = !writeMode;
                printf("COMBO ACTIVATED! Writing mode: %s\n", writeMode ? "ON" : "OFF");
                current_layer = 0;                
                // Add a longer delay here so the action doesn't spam repeatedly 
                // while the user holds the buttons down
                vTaskDelay(pdMS_TO_TICKS(500)); 
            } else {
                printf("Interrupt triggered by GPIO %"PRIu32", layer set to %d\n", io_num, current_layer);
            }

            // CRITICAL: Because all 3 switches will generate their own interrupts, 
            // the queue will fill up with redundant events. 
            // We clear the queue now so the task goes back to a clean sleep state.
            xQueueReset(gpio_evt_queue);
        }
    }
}
/**
 * @brief Reads the joystick and returns the direction index (0-7) or 9 for center.
 */
static int leer_joystick(void) {
    int x, y;
    if (adc_oneshot_read(adc1_handle, JOY_X_CHANNEL, &x) != ESP_OK) return 9;
    if (adc_oneshot_read(adc1_handle, JOY_Y_CHANNEL, &y) != ESP_OK) return 9;

    // Center calibration
    if (abs(x - 1995) < 300 && abs(y - 1882) < 300) return 9; // Center
    if (abs(x - 1995) < 300 && abs(y -    0) < 300) return 0; // Up
    if (abs(x - 4095) < 300 && abs(y -    0) < 300) return 1; // Up-Right
    if (abs(x - 4095) < 300 && abs(y - 1882) < 300) return 2; // Right
    if (abs(x - 4095) < 300 && abs(y - 4095) < 300) return 3; // Down-Right
    if (abs(x - 1995) < 300 && abs(y - 4095) < 300) return 4; // Down
    if (abs(x -    0) < 300 && abs(y - 4095) < 300) return 5; // Down-Left
    if (abs(x -    0) < 300 && abs(y - 1882) < 300) return 6; // Left
    if (abs(x -    0) < 300 && abs(y -    0) < 300) return 7; // Up-Left

    return 9; 
}

/**
 * @brief Sends a key press followed by a release.
 */
static void send_keyboard_pulse(uint8_t keycode) {
    if (!ble_hid_is_connected()) {
        ESP_LOGW(TAG, "Not connected or notifications not enabled");
        return;
    }

    uint8_t keys[6] = {0};
    keys[0] = keycode;
    
    // Key DOWN
    ble_hid_send_report(0, keys);
    vTaskDelay(pdMS_TO_TICKS(30)); // Minimum hold time for OS to register

    // Key UP
    memset(keys, 0, 6);
    ble_hid_send_report(0, keys);
}

static void joystick_task(void* arg) {
    uint8_t ultimo_keycode = 0;
    int debounce_counter = 0;

    while (1) {
        int dir = leer_joystick();

        if (dir != 9) {
            uint8_t keycode = diccionario[current_layer][dir];
            
            // Pass the caps_lock_active state to the drawing function
            if (writeMode) {
                draw_char_from_font(diccionario_char[current_layer][dir], 1, 1, 1, current_layer, caps_lock_active);
            } else {
                draw_char_from_font(diccionario_char[current_layer][dir], 1, 1, 0, current_layer, caps_lock_active);
            }
            
            // Added 0x39 to the allowed repeatable keys
            if (keycode != ultimo_keycode || keycode == 0x2C || keycode == 0x28 || keycode == 0x2A) {
                if (++debounce_counter >= 2) { 
                    ESP_LOGI(TAG, "Key Pulse: 0x%02X (Char:%s L:%d D:%d)", 
                             keycode, diccionario_char[current_layer][dir], current_layer, dir);
                    
                    if (writeMode) {
                        send_keyboard_pulse(keycode);
                        
                        // NEW: Toggle local state if Caps Lock was sent
                        if (keycode == 0x39) {
                            caps_lock_active = !caps_lock_active;
                            ESP_LOGI(TAG, "Caps Lock Toggled locally to: %d", caps_lock_active);
                        }
                    }
                    
                    ultimo_keycode = keycode;
                    debounce_counter = 0;
                }
            }
        } else {
            ultimo_keycode = 0;
            debounce_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Poll at 50Hz
    }
}

void app_main(void) {
    // 1. Initialize NVS (Required for Bluetooth bonding)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. ADC Initialization
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOY_X_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOY_Y_CHANNEL, &chan_cfg));

    // 3. GPIO Initialization
    // We use ANYEDGE so we can detect both press and release for layer switching
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOY_SW_GPIO) | (1ULL << BUTTON_1_GPIO) | (1ULL << BUTTON_2_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 4. Initialize BLE HID Stack
    ble_hid_init();
    init_led_strip();

    ESP_LOGI(TAG, "Keyboard Initialized and Advertising...");

    // 5. Create the RTOS Queue
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // 6. Spawn the FreeRTOS Tasks
    xTaskCreate(button_handler_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(joystick_task, "joystick_task", 4096, NULL, 5, NULL);

    // 7. Install and attach the ISR handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(JOY_SW_GPIO, gpio_isr_handler, (void*)(uintptr_t) JOY_SW_GPIO);
    gpio_isr_handler_add(BUTTON_1_GPIO, gpio_isr_handler, (void*)(uintptr_t) BUTTON_1_GPIO);
    gpio_isr_handler_add(BUTTON_2_GPIO, gpio_isr_handler, (void*)(uintptr_t) BUTTON_2_GPIO);
}
