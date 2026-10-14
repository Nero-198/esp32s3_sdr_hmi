/* Pin map for the Sunton ESP32-8048S050C (5" 800x480 ST7262 RGB, GT911).
 * Source: rzeldent/platformio-espressif32-sunton esp32-8048S050C.json.
 * DATA order follows esp_lcd RGB565: DATA0-4 = B, DATA5-10 = G, DATA11-15 = R.
 */
#pragma once

#define BOARD_LCD_H_RES 800
#define BOARD_LCD_V_RES 480
#define BOARD_LCD_PCLK_HZ (16 * 1000 * 1000)

#define BOARD_LCD_HSYNC_PULSE 4
#define BOARD_LCD_HSYNC_BACK 8
#define BOARD_LCD_HSYNC_FRONT 8
#define BOARD_LCD_VSYNC_PULSE 4
#define BOARD_LCD_VSYNC_BACK 8
#define BOARD_LCD_VSYNC_FRONT 8

#define BOARD_PIN_LCD_HSYNC 39
#define BOARD_PIN_LCD_VSYNC 41
#define BOARD_PIN_LCD_DE 40
#define BOARD_PIN_LCD_PCLK 42
#define BOARD_PIN_LCD_BCKL 2

#define BOARD_PIN_LCD_DATA                                    \
  { 8, 3, 46, 9, 1,        /* B0-B4 */                        \
    5, 6, 7, 15, 16, 4,    /* G0-G5 */                        \
    45, 48, 47, 21, 14 }   /* R0-R4 */

#define BOARD_PIN_TOUCH_SDA 19
#define BOARD_PIN_TOUCH_SCL 20
#define BOARD_PIN_TOUCH_INT 18
#define BOARD_PIN_TOUCH_RST 38
