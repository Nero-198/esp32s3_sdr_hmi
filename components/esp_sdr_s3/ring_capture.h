/* Shadows upstream common/ring_capture.h for the adapter build.
 *
 * Upstream reserves 0x3fcb0000-0x3fce0000 (192 KiB): the snapshot aperture
 * at 0x3fcd0000 (MAC_DUMP_USAGE=4, IQ_BUFFER) plus two banks for the
 * continuous ring. The ring is stubbed out here (ring_stub.c), so only the
 * 64 KiB snapshot aperture stays reserved and 128 KiB of internal SRAM goes
 * back to the heap. sram_guard.ld keeps static data below the aperture.
 */
#pragma once
#include_next "ring_capture.h"

#undef RING_BANK_BASE
#define RING_BANK_BASE 0x3fcd0000u
