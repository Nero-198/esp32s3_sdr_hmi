/* Upstream targets/esp32s3/sram_guard.ld with the boundary moved up to the
 * 64 KiB snapshot aperture (see ring_capture.h). 0x403c0000 is its IRAM alias. */
ASSERT(_iram_end <= 0x403c0000, "S3 RF snapshot aperture overlaps IRAM")
ASSERT(_data_end <= 0x3fcd0000, "S3 RF snapshot aperture overlaps data")
ASSERT(_bss_end <= 0x3fcd0000, "S3 RF snapshot aperture overlaps BSS")
