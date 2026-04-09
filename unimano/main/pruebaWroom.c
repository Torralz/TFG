#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "nimble.h"

// GPIO / ADC Configuration
// ESP32-C3 Pins (Using valid ADC1 channels 0-4)
#define JOY_X_CHANNEL   2               // GPIO0
#define JOY_Y_CHANNEL   3               // GPIO1
#define JOY_SW_GPIO     4               // GPIO4
#define BUTTON_1_GPIO   5               // GPIO2
#define BUTTON_2_GPIO   6               // GPIO3

static const char *TAG = "KBD_APP";
//TODO: hola
//fixme: 
//FIXME: hola
//BUG: caca
//NOTE: notifications
//WARN: vaya:
static adc_oneshot_unit_handle_t adc1_handle;

// HID Keycodes Dictionary (4 Layers x 8 Directions)
static const uint8_t diccionario[4][8] = {
    { 0x08, 0x04, 0x12, 0x16, 0x15, 0x11, 0x0C, 0x07 }, // e, a, o, s, r, n, i, d
    { 0x0F, 0x06, 0x17, 0x18, 0x10, 0x13, 0x05, 0x0A }, // l, c, u, m, p, t, b, g
    { 0x19, 0x1C, 0x14, 0x0B, 0x09, 0x1D, 0x0D, 0x33 }, // v, y, q, h, f, z, j, ñ (approx)
    { 0x1B, 0x0E, 0x1A, 0x2C, 0x2C, 0x2C, 0x2C, 0x2C }, // x, k, w, space...
};

static const char diccionario_char[4][8] = {
    { 'e', 'a', 'o', 's', 'r', 'n', 'i', 'd' }, // e, a, o, s, r, n, i, d
    { 'l', 'c', 'u', 'm', 'p', 't', 'b', 'g' }, // l, c, u, m, p, t, b, g
    { 'v', 'y', 'q', 'h', 'f', 'z', 'j', 'ñ' }, // v, y, q, h, f, z, j, ñ (approx)
    { 'x', 'k', 'w', ' ', ' ', ' ', ' ', ' ' }, // x, k, w, space...
};
/**
 * @brief Reads the joystick and returns the direction index (0-7) or 9 for center.
 * Reverted to original user calibration values.
 */
static int leer_joystick(void) {
    int x, y;
    if (adc_oneshot_read(adc1_handle, JOY_X_CHANNEL, &x) != ESP_OK) return 9;
    if (adc_oneshot_read(adc1_handle, JOY_Y_CHANNEL, &y) != ESP_OK) return 9;

    // Original user calibration logic
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
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOY_SW_GPIO) | (1ULL << BUTTON_1_GPIO) | (1ULL << BUTTON_2_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 4. Initialize BLE HID Stack
    ble_hid_init();

    ESP_LOGI(TAG, "Keyboard Initialized and Advertising...");

    // 5. Application Loop
    uint8_t ultimo_keycode = 0;
    int debounce_counter = 0;

    while (1) {
        // Read Layer Buttons (Active Low)
        int b1 = !gpio_get_level(BUTTON_1_GPIO);
        int b2 = !gpio_get_level(BUTTON_2_GPIO);
        int capa = (b1 << 1) | b2; // 0-3 layers

        int dir = leer_joystick();

        if (dir != 9) {
            uint8_t keycode = diccionario[capa][dir];
            
            // Only send if key changed or if it's a "repeatable" key (e.g. Space 0x2C)
            if (keycode != ultimo_keycode || keycode == 0x2C) {
                if (++debounce_counter >= 2) { 
                    ESP_LOGI(TAG, "Key Pulse: 0x%02X (L:%d D:%d)", keycode, capa, dir);
                    send_keyboard_pulse(keycode);
                    ultimo_keycode = keycode;
                    debounce_counter = 0;
                }
            }
        } else {
            ultimo_keycode = 0;
            debounce_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // High frequency polling
    }
}
