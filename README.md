# 3d-scanner

ESP32-S3 VL53L7CX turntable scanner firmware and raw-scan replay tools.

## Contents

- `firmware/esp32s3_vl53l7cx_model_server/esp32s3_vl53l7cx_model_server.ino`
  - ESP32-S3 web UI and scanner firmware.
  - Supports raw-hit point cloud, calibrated visual hull, no-background envelope estimation, and raw debug export.
- `tools/simulate_calibration_first_replay.py`
  - Replays exported `vl53l7cx_raw_scan*.jsonl` files.
  - Supports calibrated background subtraction and no-background raw-envelope estimation.
- `tools/analyze_raw_scan_log.py`
  - Utility for inspecting raw scan/debug logs.

## Recommended Scan Flow

1. Open the ESP32 web UI.
2. Use `perpendicular` distance projection.
3. For best geometry, run `空掃校正一圈` before scanning the object.
4. If no background scan is available, use raw-hit/no-bg output only as an estimated result.

## Replay Examples

Calibrated replay:

```bash
python tools/simulate_calibration_first_replay.py \
  --empty vl53l7cx_raw_scan-empty.jsonl \
  --object vl53l7cx_raw_scan-object.jsonl \
  --distance-model perpendicular
```

No-background replay:

```bash
python tools/simulate_calibration_first_replay.py \
  --no-bg \
  --object vl53l7cx_raw_scan-object.jsonl \
  --distance-model perpendicular
```

Raw scan logs and generated images are intentionally ignored by git.
