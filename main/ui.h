#pragma once

/* Build the analyzer screen. Call once under the LVGL lock after
 * sdr_task_start(); the UI then polls the SDR task from an LVGL timer. */
void ui_build(void);
