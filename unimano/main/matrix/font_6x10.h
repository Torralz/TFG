#include <stdint.h>
#include "led_strip.h"

// 1. Le decimos al compilador que la variable led_strip existe en otro archivo
extern led_strip_handle_t led_strip;

// 2. Prototipo de tu función para que otros archivos puedan "verla"
void draw_char_from_font(const char *letter_str, uint8_t r, uint8_t g, uint8_t b, unsigned layer, bool is_upper);
