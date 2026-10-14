#include "serial_shim.h"

#include <string.h>

#include "burst_serial.h"

static char reply_buf[256];
static size_t reply_len;

void serial_shim_begin(void) {
  reply_len = 0;
  reply_buf[0] = '\0';
}

const char *serial_shim_reply(void) { return reply_buf; }

/* Only the first line of each command's reply is kept; that is the status
 * or header line. Anything after it (binary I/Q, extra lines) is dropped. */
bool burst_serial_send(const void *data, size_t size) {
  if (memchr(reply_buf, '\n', reply_len)) return true;
  size_t room = sizeof(reply_buf) - 1 - reply_len;
  if (size > room) size = room;
  memcpy(reply_buf + reply_len, data, size);
  reply_len += size;
  reply_buf[reply_len] = '\0';
  return true;
}

size_t burst_serial_try_send(const void *data, size_t size) {
  (void)burst_serial_send(data, size);
  return size;
}

void burst_serial_init(void) {}
int burst_serial_poll_line(char *line, size_t capacity) {
  (void)line;
  (void)capacity;
  return 0;
}
burst_serial_port_t burst_serial_port(void) { return BURST_SERIAL_UART; }
unsigned burst_serial_baud(void) { return 0; }
bool burst_serial_stop_requested(void) { return false; }
