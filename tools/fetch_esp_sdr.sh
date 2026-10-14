#!/bin/sh
# Fetch ESP-SDR (GPL-3.0) at the pinned revision into third_party/esp-sdr.
# components/esp_sdr_s3 builds its S3 snapshot receiver from this checkout.
set -eu
REV=e2b35dc593a378e4c5eb69cb8db59313117c9838
DIR=$(dirname "$0")/../third_party/esp-sdr
if [ ! -d "$DIR/.git" ]; then
  git clone https://github.com/ESPARGOS/esp-sdr "$DIR"
fi
git -C "$DIR" fetch --quiet origin "$REV" 2>/dev/null || git -C "$DIR" fetch --quiet origin
git -C "$DIR" checkout --quiet "$REV"
git -C "$DIR" submodule update --init --quiet
echo "esp-sdr at $(git -C "$DIR" rev-parse --short HEAD)"
