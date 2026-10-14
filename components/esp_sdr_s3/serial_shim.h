#pragma once

/* Replacement for upstream common/burst_serial.c: no transport, text replies
 * are collected for esp_sdr_s3_command() and binary payloads are dropped
 * (the caller reads the samples from the capture buffer directly). */
void serial_shim_begin(void);
const char *serial_shim_reply(void);
