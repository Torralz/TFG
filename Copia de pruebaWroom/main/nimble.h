#ifndef NIMBLE_H
#define NIMBLE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the NimBLE stack for HID keyboard
 */
void ble_hid_init(void);

/**
 * @brief Send a HID keyboard report
 * 
 * @param modifier Mask of modifier keys (Ctrl, Shift, Alt, GUI)
 * @param keycodes Array of up to 6 keycodes being pressed
 */
void ble_hid_send_report(uint8_t modifier, uint8_t keycodes[6]);

/**
 * @brief Check if a BLE client is connected and subscribed to reports
 * 
 * @return true if connected and ready to send
 */
bool ble_hid_is_connected(void);

#endif // NIMBLE_H
