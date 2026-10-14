#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* Reset the GT911 into I2C address 0x5D and read its product ID
 * (expected "911"). product_id must hold at least 5 bytes. */
esp_err_t touch_gt911_init(char *product_id);

/* Poll one touch point. Returns true and fills x/y while a finger is down.
 * Between GT911 reports (~10 ms) the previous state is repeated, so a held
 * finger reads as continuously pressed. */
bool touch_gt911_read(uint16_t *x, uint16_t *y);
