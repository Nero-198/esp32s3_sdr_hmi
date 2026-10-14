/* Stand-in for upstream common/ring_capture.c.
 *
 * The continuous ring modes (RING, SPEC, IQS) mask interrupts for the whole
 * run and take over the second core, which cannot coexist with the RGB LCD
 * and dual-core FreeRTOS. Their ~60 KiB of static buffers would also push BSS
 * into the RF capture banks, and their two extra 64 KiB banks are no longer
 * reserved (ring_capture.h). Every ring command therefore fails with
 * RING_FAIL_ARG; the snapshot path (CAP16) is unaffected.
 */
#include <string.h>

/* Upstream header directly: ./ring_capture.h would shadow it from this directory. */
#include "common/ring_capture.h"

bool ring_capture_assist;
uint32_t ring_capture_c0_blocks;

void ring_capture_init(void) {}

void ring_capture_run(const ring_config_t *config, ring_result_t *result) {
  (void)config;
  memset(result, 0, sizeof(*result));
  result->status = RING_FAIL_ARG;
}

const uint32_t *ring_capture_bank(unsigned bank) {
  (void)bank;
  return NULL; /* only read after a successful ring run, which never happens */
}

unsigned ring_capture_rate_hz(unsigned rate) {
  return rate == 1 ? 40000000u : rate == 6 ? 16000000u : 80000000u;
}

bool ring_capture_valid_nfft(unsigned n) { (void)n; return false; }
bool ring_capture_core1_alive(void) { return false; }
bool ring_capture_dual_active(void) { return false; }
void ring_capture_set_dual(bool on) { (void)on; }
