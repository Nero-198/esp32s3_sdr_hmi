# esp32s3_sdr_hmi

[English](README.md) | 日本語

ESP32-S3 の 5" HMI ボード（800×480 RGB LCD、GT911 静電容量タッチ）だけで動く、受信専用の
2.4 GHz スペクトラムアナライザです。I/Q スナップショットは
[ESP-SDR](https://github.com/ESPARGOS/esp-sdr)（GPL-3.0）で取得し、スペクトラム計算、
スイープのスティッチ、平均化、表示はすべてボード上で行います。

状態: MVP が実機で動作中。2400-2500 MHz のスイープと 24 MHz のズーム、ウォーターフォールの上に
スペクトラム（平均 + 減衰型の最大ホールド）を表示します。マーカー、ピークサーチ、ホールド、
平均化、オート/固定レンジ、ch/MHz 軸の切り替えは、すべてタッチで操作できます。

設計の背景や開発の進め方は [Wiki](https://github.com/Nero-198/esp32s3_sdr_hmi/wiki)、
進捗は [Milestones](https://github.com/Nero-198/esp32s3_sdr_hmi/milestones) を参照してください。

## ハードウェア

| 項目 | 内容 |
| --- | --- |
| モジュール | ESP32-S3-WROOM-1（chip rev v0.2、16 MB flash、8 MB PSRAM 内蔵） |
| USB | UART0（GPIO43/44）に CH340 USB-UART。コネクタにネイティブ USB は出ていない |
| アンテナ | PCB アンテナのみ |
| ボード | Sunton ESP32-8048S050C（カラーバー、GT911 の ID "911"、タッチで確認済み） |

工場出荷時のフラッシュダンプはローカルの `backup/` にあります（git 管理外）。復元するときは
`esptool --port /dev/ttyUSB0 -b 115200 write-flash 0 backup/factory_16MB.bin` を実行します。

## 構成

- `main/`: ファームウェア。`sdr_task.c` がコア 1 でキャプチャし、`lvgl_port.c`（ダブル
  フレームバッファでティアリングなし）と `ui.c` がコア 0 で LVGL を動かす。ピン配置は `board.h`。
  スペクトラムとウォーターフォールは `LV_EVENT_RENDER_START` でフレームバッファに直接描く
  （スペクトラムは内部 RAM のバンドで合成し、ウォーターフォールは PSRAM のリングバッファから
  コピーする）。LVGL が描くのはウィジェットだけ。LVGL のメモリ確保とタスクスタックは PSRAM に
  置く（`lv_mem_psram.c`）
- `components/esp_sdr_s3/`: `third_party/esp-sdr` から無改変の ESP-SDR S3 受信機をビルドし、
  `esp_sdr_s3_command()` / `esp_sdr_s3_capture()` として公開するアダプタ。上流のシリアル転送と
  連続リングキャプチャは `serial_shim.c` と `ring_stub.c` で置き換えている（リングキャプチャは
  割り込みを禁止してコア 1 を占有し、そのバッファが BSS を RF キャプチャ領域に押し出すため）。
  `ring_capture.h` と `sram_guard.ld` で予約 SRAM を 192 KiB から 64 KiB のスナップショット領域に
  縮め、フックで近い LO 間の RX DC キャリブレーションを使い回す
- `tools/fetch_esp_sdr.sh`: ESP-SDR を固定リビジョンで取得する（最初に 1 回だけ実行）
- `tools/iq_probe.py`: 標準の ESP-SDR ファームウェア用のホストツール（UART で I/Q を受け取り、
  スペクトラムを描く）
- `components/sdr_dsp/`: ハードウェアに依存しない C99 の DSP（FFT、Welch 法による dBFS
  スペクトラム、DC の穴埋め、スイープ計画とスティッチ、平均化、ピークホールド、オートレンジ、
  Wi-Fi チャンネル軸とマーカーの補助）
- `tests/host/`: `sdr_dsp` のホスト用ユニットテスト

## ホストテスト

```sh
cmake -S tests/host -B build-host -G Ninja
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

## ファームウェアのビルド

```sh
tools/fetch_esp_sdr.sh
. ~/esp/esp-idf-sdr/export.sh
idf.py -B build build
idf.py -B build -p /dev/ttyUSB0 -b 115200 flash monitor
```

USB/IP（WSL）経由だと CH340 は約 230 kbaud を超えるとバイトを落とすため、書き込みは 115200 で
行います。

## 実機での確認メモ

2026-10-15、上記のボードを机上に置き、PCB アンテナで、近くに Wi-Fi AP がある環境:

- 標準の ESP-SDR を `tools/iq_probe.py --slow` で確認: `BANDWIDTH 13` では LO に追従する
  約 20 MHz のノイズの台地（受信機の通過帯域）が見え、`BANDWIDTH 69` では 80 MHz にわたって
  ほぼ平坦。30 スナップショットの最大ホールドで、約 20 MHz 幅の Wi-Fi の山と、平均トレースより
  10-20 dB 高い狭帯域の Bluetooth のホッピングが見える
- 共存テスト（`main/main.c`、80 MS/s × 2 ステップ、ゲイン 50、1024 点 FFT）: LCD を動かしたまま
  150 スイープでキャプチャ失敗 0。スイープ時間は LCD off で 142 ms、on で 179 ms（5.6 スイープ/秒）。
  20 パーセンタイルのフロアは -58.0 / -56.2 dBFS。単純に全画面を再描画すると、さらに約 48 ms かかる
- LVGL UI（ダブルバッファ、SDR はコア 1）: ちらつきなし、スイープモードで約 6 fps。マーカー、
  軸の切り替え、ZOOM/FULL、PEAK、HOLD、AVG、RANGE はすべてタッチで動作
- 高速化（スイープモード、fps はステータス行の値）:

  | 変更 | SDR pass | UI frame | fps |
  | --- | --- | --- | --- |
  | 変更前 | 113 ms | 156 ms | 6.3 |
  | RF 予約領域 192 → 64 KiB、D-cache 64 KiB / 64 B ライン | 115 ms | 94 ms | 8.8 |
  | esp-dsp の FFT（起動時セルフテスト: ビンと 0 dBFS が一致） | 102 ms | 105 ms | 9.4 |
  | スイープの LO 間で RX DC キャリブレーションを再利用 | 45 ms | 105 ms | ~9.5 |
  | プロットをキャンバスではなくフレームバッファに直接描く | 45 ms | 63 ms | 15.8 |
  | スペクトラムを内部 RAM のバンドで合成 | 45 ms | 50 ms | 19.5 |

  DC 再利用の A/B 比較（30 パスごとに切り替え、10 ラウンド）: 再同調は 1 パスあたり 66-73 ms
  から 9 ms に短縮。DC オフセットは約 1.0 / 約 0.8 LSB、p20 フロアは -55..-59 / -55..-60 dBFS で、
  どちらもキャプチャ失敗は 0。

## ツールチェーン

ESP-SDR は ESP-IDF のコミット `25fe69f946311abdaf9ad56591f25fedbc20ac98`（v6.1-dev）を前提にして
おり、`~/esp/esp-idf-sdr` にインストールしています。

## ライセンス

ファームウェアが ESP-SDR をリンクするため、GPL-3.0 です。
