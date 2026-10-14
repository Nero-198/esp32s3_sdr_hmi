/* LVGL glue after ESP-IDF examples/peripherals/lcd/rgb_panel (double FB). */
#include "lvgl_port.h"

#include "board.h"
#include "esp_check.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "lvgl.h"
#include "touch_gt911.h"

#define LVGL_TASK_CORE 0
#define LVGL_TASK_PRIO 4
#define LVGL_TASK_STACK (8 * 1024)
#define LVGL_MIN_DELAY_MS 2
#define LVGL_MAX_DELAY_MS 30

static const char *TAG = "lvgl_port";
static SemaphoreHandle_t lock;
static TaskHandle_t task;

void lvgl_port_lock(void) { xSemaphoreTakeRecursive(lock, portMAX_DELAY); }
void lvgl_port_unlock(void) { xSemaphoreGiveRecursive(lock); }

static uint32_t tick_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static bool IRAM_ATTR on_frame_buf_complete(esp_lcd_panel_handle_t panel,
                                            const esp_lcd_rgb_panel_event_data_t *edata,
                                            void *user_ctx) {
  BaseType_t woken = pdFALSE;
  if (task) vTaskNotifyGiveFromISR(task, &woken);
  return woken == pdTRUE;
}

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  /* Direct mode: px_map is a whole frame buffer. Swap only after the last
   * dirty rectangle of the frame. */
  if (!lv_display_flush_is_last(disp)) {
    lv_display_flush_ready(disp);
    return;
  }
  ulTaskNotifyTake(pdTRUE, 0);
  esp_lcd_panel_draw_bitmap(lv_display_get_user_data(disp), 0, 0, BOARD_LCD_H_RES,
                            BOARD_LCD_V_RES, px_map);
}

static void flush_wait_cb(lv_display_t *disp) {
  /* Do not render into the buffer the DMA is still scanning out. */
  if (lv_display_flush_is_last(disp)) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  lv_display_flush_ready(disp);
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
  uint16_t x, y;
  bool down = touch_gt911_read(&x, &y);
  data->point.x = x;
  data->point.y = y;
  data->state = down ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

static void lvgl_task(void *arg) {
  for (;;) {
    lvgl_port_lock();
    uint32_t next = lv_timer_handler();
    lvgl_port_unlock();
    if (next < LVGL_MIN_DELAY_MS) next = LVGL_MIN_DELAY_MS;
    if (next > LVGL_MAX_DELAY_MS) next = LVGL_MAX_DELAY_MS;
    vTaskDelay(pdMS_TO_TICKS(next));
  }
}

esp_err_t lvgl_port_start(esp_lcd_panel_handle_t panel, void (*build_ui)(void)) {
  lock = xSemaphoreCreateRecursiveMutex();
  ESP_RETURN_ON_FALSE(lock, ESP_ERR_NO_MEM, TAG, "lock");

  lv_init();
  lv_tick_set_cb(tick_ms);
  lv_display_t *disp = lv_display_create(BOARD_LCD_H_RES, BOARD_LCD_V_RES);
  lv_display_set_user_data(disp, panel);
  lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
  void *fb1 = NULL, *fb2 = NULL;
  ESP_RETURN_ON_ERROR(esp_lcd_rgb_panel_get_frame_buffer(panel, 2, &fb1, &fb2), TAG, "fbs");
  lv_display_set_buffers(disp, fb1, fb2, BOARD_LCD_H_RES * BOARD_LCD_V_RES * 2,
                         LV_DISPLAY_RENDER_MODE_DIRECT);
  lv_display_set_flush_cb(disp, flush_cb);
  lv_display_set_flush_wait_cb(disp, flush_wait_cb);
  esp_lcd_rgb_panel_event_callbacks_t cbs = {.on_frame_buf_complete = on_frame_buf_complete};
  ESP_RETURN_ON_ERROR(esp_lcd_rgb_panel_register_event_callbacks(panel, &cbs, disp), TAG, "cbs");

  lv_indev_t *touch = lv_indev_create();
  lv_indev_set_type(touch, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(touch, touch_read_cb);

  lvgl_port_lock();
  build_ui();
  lvgl_port_unlock();

  /* Internal RAM is nearly exhausted (ESP-SDR reserves 192 KiB); the stack
   * goes to PSRAM. */
  BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(lvgl_task, "lvgl", LVGL_TASK_STACK, NULL,
                                                  LVGL_TASK_PRIO, &task, LVGL_TASK_CORE,
                                                  MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
