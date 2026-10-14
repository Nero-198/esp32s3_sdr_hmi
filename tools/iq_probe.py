#!/usr/bin/env python3
"""Capture I/Q snapshots from stock ESP-SDR firmware over UART and report the spectrum.

Used to validate the RF path on the HMI board before the display firmware exists.
Example:
    python tools/iq_probe.py --port /dev/ttyUSB0 --freq 2437 --rate 0 --png spec.png
"""
from __future__ import annotations

import argparse
import sys
import time
import zlib

import numpy as np
import serial

RATES_HZ = {0: 80e6, 1: 40e6, 6: 16e6}


def read_line(ser: serial.Serial, timeout: float = 3.0) -> str:
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            return buf.decode(errors="replace").strip()
        buf += b
    raise TimeoutError(f"no reply (partial: {bytes(buf)!r})")


def command(ser: serial.Serial, line: str) -> str:
    ser.write((line + "\n").encode())
    return read_line(ser)


def resync(ser: serial.Serial) -> None:
    ser.reset_input_buffer()
    nonce = str(int(time.monotonic() * 1000) % 100000)
    ser.write(f"SYNC {nonce}\n".encode())
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        try:
            if nonce in read_line(ser, 1.0):
                return
        except TimeoutError:
            pass
    raise TimeoutError("SYNC failed")


def capture(ser: serial.Serial, samples: int, rate: int) -> np.ndarray:
    """Return complex64 samples from one CAP16 snapshot, or raise on CRC/framing errors."""
    header = command(ser, f"CAP16 {samples} {rate}")
    parts = header.split()
    if len(parts) != 4 or parts[0] != "DATA":
        raise ValueError(f"unexpected reply: {header}")
    n, crc = int(parts[1]), int(parts[2], 16)
    want = 2 * n
    payload = ser.read(want)
    if len(payload) != want:
        raise ValueError(f"short payload {len(payload)}/{want}")
    if zlib.crc32(payload) != crc:
        raise ValueError("CRC mismatch")
    raw = np.frombuffer(payload, dtype=np.int8).astype(np.float32)
    return raw[0::2] + 1j * raw[1::2]


def spectrum_dbfs(iq: np.ndarray, fft_n: int) -> np.ndarray:
    """Welch Hann spectrum in dBFS (full-scale complex tone = 0 dBFS), DC-centred.

    Mirrors components/sdr_dsp/sdr_spectrum.c so host and device numbers agree.
    """
    segs = len(iq) // fft_n
    win = 0.5 - 0.5 * np.cos(2 * np.pi * np.arange(fft_n) / fft_n)
    x = iq[: segs * fft_n].reshape(segs, fft_n) * win
    p = np.mean(np.abs(np.fft.fft(x, axis=1)) ** 2, axis=0)
    p /= (127.0 * win.sum()) ** 2
    return np.fft.fftshift(10 * np.log10(np.maximum(p, 1e-20)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=2_000_000)
    ap.add_argument("--freq", type=int, default=2437, help="LO frequency in MHz")
    ap.add_argument("--rate", type=int, default=0, choices=sorted(RATES_HZ))
    ap.add_argument("--samples", type=int, default=16380)
    ap.add_argument("--fft", type=int, default=1024)
    ap.add_argument("--count", type=int, default=5, help="snapshots to average")
    ap.add_argument("--retries", type=int, default=5)
    ap.add_argument("--slow", action="store_true",
                    help="switch the session to 1 Mbaud (helps over USB/IP)")
    ap.add_argument("--cmd", action="append", default=[],
                    help="extra command sent after FREQ, e.g. 'BANDWIDTH 13' (repeatable)")
    ap.add_argument("--hold", action="store_true",
                    help="also plot the per-bin maximum over snapshots (bursty signals)")
    ap.add_argument("--png", help="save a spectrum plot")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        time.sleep(0.2)
        try:
            resync(ser)
        except TimeoutError:
            # BAUD is session-only; a previous run may have left the board at 1 Mbaud.
            ser.baudrate = 1_000_000 if args.baud != 1_000_000 else 2_000_000
            resync(ser)
        if args.slow and ser.baudrate != 1_000_000:
            print(f"{'BAUD':>10} -> {command(ser, 'BAUD 1000000')}")
            time.sleep(0.05)
            ser.baudrate = 1_000_000
            resync(ser)
        for q in ("INFO", "TRANSPORT?", "VERSION?", "LIMITS?", "RANGE?"):
            print(f"{q:>10} -> {command(ser, q)}")
        print(f"{'FREQ':>10} -> {command(ser, f'FREQ {args.freq}')}")
        for extra in args.cmd:
            print(f"{extra:>10} -> {command(ser, extra)}")

        specs, errors, durations = [], 0, []
        while len(specs) < args.count:
            t0 = time.monotonic()
            try:
                iq = capture(ser, args.samples, args.rate)
            except (ValueError, TimeoutError) as e:
                errors += 1
                print(f"capture error: {e}", file=sys.stderr)
                if errors > args.retries:
                    return 1
                resync(ser)
                continue
            durations.append(time.monotonic() - t0)
            specs.append(spectrum_dbfs(iq, args.fft))
            print(f"snapshot {len(specs)}: rms={np.sqrt(np.mean(np.abs(iq) ** 2)):.1f} LSB, "
                  f"clipped={np.mean(np.abs(iq.real) >= 127) * 100:.2f}%")

    fs = RATES_HZ[args.rate]
    spec = np.mean(specs, axis=0)
    freqs = args.freq * 1e6 + (np.arange(args.fft) - args.fft // 2) * fs / args.fft
    floor = np.percentile(spec, 20)
    pk = int(np.argmax(spec))
    print(f"errors={errors}, mean transfer {np.mean(durations) * 1000:.0f} ms/snapshot")
    print(f"noise floor (p20) {floor:.1f} dBFS, peak {spec[pk]:.1f} dBFS at {freqs[pk] / 1e6:.2f} MHz")

    if args.png:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        plt.figure(figsize=(10, 4))
        plt.plot(freqs / 1e6, spec, lw=0.8, label="average")
        if args.hold:
            plt.plot(freqs / 1e6, np.max(specs, axis=0), lw=0.8, label="max hold")
            plt.legend()
        plt.xlabel("MHz")
        plt.ylabel("dBFS")
        plt.title(f"ESP-SDR S3 LO {args.freq} MHz, {fs / 1e6:.0f} MS/s, {len(specs)} snapshots")
        plt.grid(alpha=0.3)
        plt.tight_layout()
        plt.savefig(args.png, dpi=100)
        print(f"saved {args.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
