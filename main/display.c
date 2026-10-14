#include "display.h"

#include "board.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_panel_rgb.h"

static const char *TAG = "display";

esp_err_t display_init(esp_lcd_panel_handle_t *panel) {
  gpio_config_t bl = {
      .pin_bit_mask = 1ULL << BOARD_PIN_LCD_BCKL,
      .mode = GPIO_MODE_OUTPUT,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&bl), TAG, "backlight gpio");
  gpio_set_level(BOARD_PIN_LCD_BCKL, 0);

  esp_lcd_rgb_panel_config_t cfg = {
      .clk_src = LCD_CLK_SRC_DEFAULT,
      .data_width = 16,
      .in_color_format = LCD_COLOR_FMT_RGB565,
      .out_color_format = LCD_COLOR_FMT_RGB565,
      .num_fbs = 2,
      .bounce_buffer_size_px = 10 * BOARD_LCD_H_RES,
      .dma_burst_size = 64,
      .hsync_gpio_num = BOARD_PIN_LCD_HSYNC,
      .vsync_gpio_num = BOARD_PIN_LCD_VSYNC,
      .de_gpio_num = BOARD_PIN_LCD_DE,
      .pclk_gpio_num = BOARD_PIN_LCD_PCLK,
      .disp_gpio_num = -1,
      .data_gpio_nums = BOARD_PIN_LCD_DATA,
      .timings =
          {
              .pclk_hz = BOARD_LCD_PCLK_HZ,
              .h_res = BOARD_LCD_H_RES,
              .v_res = BOARD_LCD_V_RES,
              .hsync_pulse_width = BOARD_LCD_HSYNC_PULSE,
              .hsync_back_porch = BOARD_LCD_HSYNC_BACK,
              .hsync_front_porch = BOARD_LCD_HSYNC_FRONT,
              .vsync_pulse_width = BOARD_LCD_VSYNC_PULSE,
              .vsync_back_porch = BOARD_LCD_VSYNC_BACK,
              .vsync_front_porch = BOARD_LCD_VSYNC_FRONT,
              .flags.pclk_active_neg = 1,
          },
      .flags.fb_in_psram = 1,
  };
  ESP_RETURN_ON_ERROR(esp_lcd_new_rgb_panel(&cfg, panel), TAG, "new rgb panel");
  ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(*panel), TAG, "reset");
  ESP_RETURN_ON_ERROR(esp_lcd_panel_init(*panel), TAG, "init");
  gpio_set_level(BOARD_PIN_LCD_BCKL, 1);
  return ESP_OK;
}
