# esp32s3_sdr_hmi

Standalone, receive-only 2.4 GHz spectrum analyzer for an ESP32-S3 5" HMI
board (800×480 RGB LCD, GT911 capacitive touch). I/Q snapshots come from
[ESP-SDR](https://github.com/ESPARGOS/esp-sdr) (GPL-3.0); spectrum,
sweep stitching, averaging and display run on the board itself.

Status: MVP running on hardware. Sweep 2400-2500 MHz or 24 MHz zoom, spectrum
(average + decaying max hold) over a waterfall, marker, peak search, hold,
averaging, auto/fixed range and a ch/MHz axis, all by touch.

## Hardware

| Item | Value |
| --- | --- |
| Module | ESP32-S3-WROOM-1 (chip rev v0.2, 16 MB flash, 8 MB embedded PSRAM) |
| USB | CH340 USB-UART on UART0 (GPIO43/44); no native USB on the connector |
| Antenna | PCB antenna only |
| Board | Sunton ESP32-8048S050C (confirmed: colour bars, GT911 ID "911", touch) |

A factory flash dump is kept locally in `backup/` (git-ignored). Restore with
`esptool --port /dev/ttyUSB0 -b 115200 write-flash 0 backup/factory_16MB.bin`.

## Layout

- `main/`: firmware. `sdr_task.c` acquires on core 1; `lvgl_port.c` (double
  frame buffer, no tearing) and `ui.c` run LVGL on core 0; `board.h` has the pin map.
  Spectrum and waterfall are painted straight into the frame buffer at
  `LV_EVENT_RENDER_START` (spectrum composed in internal-RAM bands, waterfall from
  a PSRAM ring); LVGL only draws the widgets. LVGL allocations and its task
  stack live in PSRAM (`lv_mem_psram.c`)
- `components/esp_sdr_s3/`: adapter that builds the unmodified ESP-SDR S3 receiver
  from `third_party/esp-sdr` and exposes `esp_sdr_s3_command()` / `esp_sdr_s3_capture()`.
  Upstream serial transport and continuous ring capture are replaced by
  `serial_shim.c` and `ring_stub.c` (the ring masks interrupts, owns core 1 and
  its buffers push BSS into the RF capture banks). `ring_capture.h` and
  `sram_guard.ld` shrink the reserved SRAM from 192 KiB to the 64 KiB snapshot
  aperture, and a hook reuses RX DC calibration across nearby LOs
- `tools/fetch_esp_sdr.sh`: fetches ESP-SDR at the pinned revision (run once)
- `tools/iq_probe.py`: host tool for stock ESP-SDR firmware (I/Q over UART, spectrum plot)
- `components/sdr_dsp/`: plain C99 DSP with no hardware dependencies (FFT, Welch
  dBFS spectrum, DC fill, sweep plan and stitching, averaging, peak hold,
  auto range, Wi-Fi channel axis and marker helpers)
- `tests/host/`: host unit tests for `sdr_dsp`

## Host tests

```sh
cmake -S tests/host -B build-host -G Ninja
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

## Firmware build

```sh
tools/fetch_esp_sdr.sh
. ~/esp/esp-idf-sdr/export.sh
idf.py -B build build
idf.py -B build -p /dev/ttyUSB0 -b 115200 flash monitor
```

Over USB/IP (WSL) the CH340 drops bytes above ~230 kbaud; flash at 115200.

## Manual validation notes

2026-10-15, board above on the desk, PCB antenna, Wi-Fi AP nearby:

- Stock ESP-SDR via `tools/iq_probe.py --slow`: `BANDWIDTH 13` shows a ~20 MHz
  noise plateau that follows the LO (receiver passband); `BANDWIDTH 69` is
  roughly flat over 80 MHz. Max hold over 30 snapshots shows ~20 MHz Wi-Fi
  humps and narrow Bluetooth hops 10-20 dB above the average trace.
- Coexistence test (`main/main.c`, 2 x 80 MS/s steps, gain 50, 1024-point FFT):
  0 capture failures in 150 sweeps with the LCD running. Sweep 142 ms with the
  LCD off vs 179 ms on (5.6 sweeps/s); 20th-percentile floor -58.0 vs
  -56.2 dBFS. Naive full-area redraw costs another ~48 ms.
- LVGL UI (double buffer, SDR on core 1): no flicker, ~6 fps in sweep mode;
  marker, axis toggle, ZOOM/FULL, PEAK, HOLD, AVG, RANGE all work by touch.
- Speed-up (sweep mode, fps from the status line):

  | Change | SDR pass | UI frame | fps |
  | --- | --- | --- | --- |
  | baseline | 113 ms | 156 ms | 6.3 |
  | RF reservation 192 -> 64 KiB, D-cache 64 KiB / 64 B lines | 115 ms | 94 ms | 8.8 |
  | esp-dsp FFT (boot self-test: bin and 0 dBFS match) | 102 ms | 105 ms | 9.4 |
  | reuse RX DC calibration across sweep LOs | 45 ms | 105 ms | ~9.5 |
  | paint plots into the frame buffer instead of canvases | 45 ms | 63 ms | 15.8 |
  | compose spectrum in internal-RAM bands | 45 ms | 50 ms | 19.5 |

  DC reuse A/B (alternating every 30 passes, 10 rounds): retune 66-73 ms ->
  9 ms per pass; DC offset ~1.0 vs ~0.8 LSB, p20 floor -55..-59 vs -55..-60
  dBFS, 0 capture failures either way.

## Toolchain

ESP-SDR pins ESP-IDF commit `25fe69f946311abdaf9ad56591f25fedbc20ac98`
(v6.1-dev), installed at `~/esp/esp-idf-sdr`.

## License

GPL-3.0, because the firmware links ESP-SDR.
