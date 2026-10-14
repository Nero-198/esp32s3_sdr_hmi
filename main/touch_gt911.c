#include "touch_gt911.h"

#include <string.h>

#include "board.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define GT911_ADDR 0x5D
#define GT911_REG_PRODUCT_ID 0x8140
#define GT911_REG_STATUS 0x814E
#define GT911_REG_POINT1 0x8150

static const char *TAG = "gt911";
static i2c_master_dev_handle_t dev;

static esp_err_t reg_read(uint16_t reg, uint8_t *data, size_t len) {
  uint8_t addr[2] = {reg >> 8, reg & 0xff};
  return i2c_master_transmit_receive(dev, addr, 2, data, len, 50);
}

static esp_err_t reg_write8(uint16_t reg, uint8_t v) {
  uint8_t buf[3] = {reg >> 8, reg & 0xff, v};
  return i2c_master_transmit(dev, buf, 3, 50);
}

esp_err_t touch_gt911_init(char *product_id) {
  /* INT held low across the reset release selects address 0x5D. */
  gpio_config_t out = {
      .pin_bit_mask = (1ULL << BOARD_PIN_TOUCH_RST) | (1ULL << BOARD_PIN_TOUCH_INT),
      .mode = GPIO_MODE_OUTPUT,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&out), TAG, "reset gpio");
  gpio_set_level(BOARD_PIN_TOUCH_INT, 0);
  gpio_set_level(BOARD_PIN_TOUCH_RST, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(BOARD_PIN_TOUCH_RST, 1);
  vTaskDelay(pdMS_TO_TICKS(5));
  gpio_set_direction(BOARD_PIN_TOUCH_INT, GPIO_MODE_INPUT);
  vTaskDelay(pdMS_TO_TICKS(50));

  i2c_master_bus_config_t bus_cfg = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = BOARD_PIN_TOUCH_SDA,
      .scl_io_num = BOARD_PIN_TOUCH_SCL,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  i2c_master_bus_handle_t bus;
  ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG, "i2c bus");
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = GT911_ADDR,
      .scl_speed_hz = 400000,
  };
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &dev), TAG, "i2c dev");

  uint8_t id[4] = {0};
  ESP_RETURN_ON_ERROR(reg_read(GT911_REG_PRODUCT_ID, id, sizeof(id)), TAG, "product id");
  memcpy(product_id, id, 4);
  product_id[4] = '\0';
  return ESP_OK;
}

bool touch_gt911_read(uint16_t *x, uint16_t *y) {
  static bool down;
  static uint16_t last_x, last_y;
  uint8_t status = 0;
  if (reg_read(GT911_REG_STATUS, &status, 1) == ESP_OK && (status & 0x80)) {
    down = false;
    if ((status & 0x0f) > 0) {
      uint8_t p[4];
      if (reg_read(GT911_REG_POINT1, p, sizeof(p)) == ESP_OK) {
        last_x = (uint16_t)(p[0] | (p[1] << 8));
        last_y = (uint16_t)(p[2] | (p[3] << 8));
        down = true;
      }
    }
    reg_write8(GT911_REG_STATUS, 0);
  }
  *x = last_x;
  *y = last_y;
  return down;
}
