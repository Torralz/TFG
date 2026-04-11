#ifndef MATRIX_H
#define MATRIX_H

#include <stdint.h>

// 1. Configuration Macros
// Exposed here so other files can know the pin and pixel count if needed
#define LED_STRIP_GPIO_PIN   10  // D0 on XIAO ESP32-C3
#define LED_STRIP_NUM_PIXELS 60

// 2. Global Variables
// Using 'extern' tells the compiler these variables exist in the .c file.
// This allows other files to read or change the brightness and refresh rate.
extern uint8_t REFRESH_RATE;
extern uint8_t BRIGHTNESS;

// 3. Function Prototypes
// This allows other files (like your main.c) to call these functions
void init_led_strip(void);
void leds_task(void);
void button_task(void);
void brightness_task(void);
void writing_task(const char *text);

#endif // XIAO_MATRIX_H
