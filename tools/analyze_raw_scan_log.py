#!/usr/bin/env python3
"""Analyze exported VL53L7CX raw 360-degree scan logs.

Input is either:
- vl53l7cx_raw_scan.jsonl
- vl53l7cx_debug_bundle.txt

The parser reads lines prefixed with `raw_meta ` and `raw_frame `. It then
replays several scan geometry variants against the known 28 x 43 mm rectangle
so the bad formula can be isolated from the same raw data.
"""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
import math
from pathlib import Path
from typing import Iterable


DEFAULT_SENSOR_Z_MM = 61.0
DEFAULT_HFOV_DEG = 60.0
DEFAULT_VFOV_DEG = 60.0
DEFAULT_CENTER_D_MM = 75.0
DEFAULT_Z_FLOOR_MM = 18.0
DEFAULT_Z_CEIL_MM = 150.0
DEFAULT_SUPPORT_BIN_DEG = 3.0
DEFAULT_EXPECTED_SHORT_MM = 28.0
DEFAULT_EXPECTED_LONG_MM = 43.0


@dataclass
class Frame:
    frame_id: int
    angle: float
    dist: list[int]
    status: list[int]
    cal: dict


@dataclass
class Support:
    angle: float
    h: float
    nx: float
    ny: float
    samples: list[float]
    count: int = 1


@dataclass
class Fit:
    cx: float
    cy: float
    hx: float
    hy: float
    phi_deg: float
    err: float
    supports: int

    @property
    def sorted_size(self) -> tuple[float, float]:
        return tuple(sorted((abs(self.hx) * 2.0, abs(self.hy) * 2.0)))


def num(v: object, fallback: float = 0.0) -> float:
    try:
        out = float(v)
    except (TypeError, ValueError):
        return fallback
    return out if math.isfinite(out) else fallback


def norm_angle(v: float) -> float:
    return v % 360.0


def clockwise_delta(start: float, end: float) -> float:
    return (end - start + 360.0) % 360.0


def rotate2(x: float, y: float, deg: float) -> tuple[float, float]:
    a = math.radians(deg)
    c = math.cos(a)
    s = math.sin(a)
    return c * x - s * y, s * x + c * y


def zone_row_col(zone: int, mapping: str) -> tuple[int, int]:
    row = zone // 8
    col = zone % 8
    if mapping == "identity":
        return row, col
    if mapping == "transpose":
        return col, row
    if mapping == "flip_x":
        return row, 7 - col
    if mapping == "flip_y":
        return 7 - row, col
    if mapping == "rot180":
        return 7 - row, 7 - col
    if mapping == "transpose_flip_x":
        return col, 7 - row
    if mapping == "transpose_flip_y":
        return 7 - col, row
    raise ValueError(f"unknown zone mapping: {mapping}")


def ray_dir(row: int, col: int, hfov_deg: float, vfov_deg: float) -> tuple[float, float, float]:
    h = ((col + 0.5) / 8.0 - 0.5) * math.radians(hfov_deg)
    v = (0.5 - (row + 0.5) / 8.0) * math.radians(vfov_deg)
    x = math.sin(h) * math.cos(v)
    y = -math.cos(h) * math.cos(v)
    z = math.sin(v)
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    return x / n, y / n, z / n


def usable_status(status: int) -> bool:
    return status in (5, 6, 9)


def parse_log(path: Path) -> tuple[dict, list[Frame]]:
    meta: dict = {}
    frames: list[Frame] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("raw_meta "):
            try:
                meta = json.loads(line[len("raw_meta ") :])
            except json.JSONDecodeError:
                pass
        elif line.startswith("raw_frame "):
            payload = json.loads(line[len("raw_frame ") :])
            dist = [int(v or 0) for v in payload.get("dist", [])[:64]]
            status = [int(v or 0) for v in payload.get("status", [])[:64]]
            if len(dist) < 64:
                dist.extend([0] * (64 - len(dist)))
            if len(status) < 64:
                status.extend([255] * (64 - len(status)))
            frames.append(
                Frame(
                    frame_id=int(payload.get("frame") or len(frames) + 1),
                    angle=num(payload.get("server_angle")),
                    dist=dist,
                    status=status,
                    cal=payload.get("cal") or {},
                )
            )
    return meta, frames


def percentile(values: list[float], q: float) -> float | None:
    vals = sorted(v for v in values if math.isfinite(v))
    if not vals:
        return None
    idx = max(0, min(len(vals) - 1, round((len(vals) - 1) * q)))
    return vals[idx]


def support_y(values: Iterable[float], mode: str) -> float | None:
    vals = sorted(v for v in values if math.isfinite(v))
    if not vals:
        return None
    if mode == "old_percentile":
        if len(vals) == 1:
            return vals[0]
        if len(vals) == 2:
            return sum(vals) / 2.0
        return percentile(vals, 0.68 if len(vals) >= 5 else 0.62)
    if mode == "front_high_mean":
        keep = 2 if len(vals) >= 5 else 1
        return sum(vals[-keep:]) / keep
    if mode == "front_max":
        return vals[-1]
    if mode == "front_top3":
        keep = min(3, len(vals))
        return sum(vals[-keep:]) / keep
    raise ValueError(f"unknown support mode: {mode}")


def robust_sample(samples: list[float], mode: str) -> float:
    vals = sorted(v for v in samples if math.isfinite(v))
    if not vals:
        return 0.0
    if mode == "old":
        q = 0.62 if len(vals) < 5 else 0.65
    elif mode == "new":
        q = 0.85 if len(vals) < 5 else 0.80
    else:
        raise ValueError(f"unknown robust mode: {mode}")
    return vals[max(0, min(len(vals) - 1, round((len(vals) - 1) * q)))]


def collect_hits(
    frame: Frame,
    *,
    mapping: str,
    min_dist: float,
    max_dist: float,
    center_d: float,
    sensor_z: float,
    hfov_deg: float,
    vfov_deg: float,
    z_floor: float,
    z_ceil: float,
    radius: float,
) -> list[tuple[int, int, int, float, float, float, int]]:
    trusted = sum(1 for i, d in enumerate(frame.dist) if min_dist < d < max_dist and usable_status(frame.status[i]))
    require_status = trusted >= 2
    out = []
    for zone, d in enumerate(frame.dist):
        if not (min_dist < d < max_dist):
            continue
        if require_status and not usable_status(frame.status[zone]):
            continue
        row, col = zone_row_col(zone, mapping)
        dx, dy, dz = ray_dir(row, col, hfov_deg, vfov_deg)
        x = dx * d
        y = center_d + dy * d
        z = sensor_z + dz * d
        if z < z_floor or z > z_ceil:
            continue
        if abs(x) > radius + 35 or y < -radius - 25 or y > radius + 25:
            continue
        out.append((zone, row, col, x, y, z, d))
    return out


def build_supports(
    frames: list[Frame],
    *,
    mapping: str,
    support_mode: str,
    robust_mode: str,
    min_dist: float,
    max_dist: float,
    center_d: float,
    sensor_z: float,
    hfov_deg: float,
    vfov_deg: float,
    z_floor: float,
    z_ceil: float,
    radius: float,
    dir_sign: float,
    angle_scale: float,
    support_bin_deg: float,
) -> dict[int, Support]:
    supports: dict[int, Support] = {}
    zero_angle: float | None = None
    for frame in frames:
        hits = collect_hits(
            frame,
            mapping=mapping,
            min_dist=min_dist,
            max_dist=max_dist,
            center_d=center_d,
            sensor_z=sensor_z,
            hfov_deg=hfov_deg,
            vfov_deg=vfov_deg,
            z_floor=z_floor,
            z_ceil=z_ceil,
            radius=radius,
        )
        if len(hits) < 2:
            continue
        mid = [p for p in hits if 2 <= p[1] <= 5]
        if len(mid) < 2:
            mid = hits
        h = support_y((p[4] for p in mid), support_mode)
        if h is None:
            continue
        if zero_angle is None:
            zero_angle = frame.angle
        fixed_angle = dir_sign * clockwise_delta(zero_angle, frame.angle) * angle_scale
        nx, ny = rotate2(0.0, 1.0, fixed_angle)
        bin_id = round(fixed_angle / support_bin_deg)
        old = supports.get(bin_id)
        if old is None:
            supports[bin_id] = Support(fixed_angle, h, nx, ny, [h], 1)
        else:
            old.samples.append(h)
            if len(old.samples) > 24:
                old.samples.pop(0)
            old.h = robust_sample(old.samples, robust_mode)
            old.nx = nx
            old.ny = ny
            old.angle = fixed_angle
            old.count += 1
    return supports


def solve4(mat: list[list[float]], vec: list[float]) -> list[float] | None:
    m = [row[:] + [vec[i]] for i, row in enumerate(mat)]
    for col in range(4):
        piv = max(range(col, 4), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-9:
            return None
        if piv != col:
            m[piv], m[col] = m[col], m[piv]
        div = m[col][col]
        for c in range(col, 5):
            m[col][c] /= div
        for r in range(4):
            if r == col:
                continue
            f = m[r][col]
            for c in range(col, 5):
                m[r][c] -= f * m[col][c]
    return [m[i][4] for i in range(4)]


def fit_rectangle(supports: dict[int, Support]) -> Fit | None:
    cs = [s for s in supports.values() if math.isfinite(s.h)]
    if len(cs) < 8:
        return None
    best: Fit | None = None
    for deg in range(180):
        phi = math.radians(deg)
        e1 = (math.cos(phi), math.sin(phi))
        e2 = (-math.sin(phi), math.cos(phi))
        mat = [[0.0] * 4 for _ in range(4)]
        vec = [0.0] * 4
        weight_sum = 0.0
        for s in cs:
            a1 = abs(s.nx * e1[0] + s.ny * e1[1])
            a2 = abs(s.nx * e2[0] + s.ny * e2[1])
            row = [s.nx, s.ny, a1, a2]
            weight = min(4.0, 1.0 + math.log2((s.count or 1) + 1.0))
            weight_sum += weight
            for i in range(4):
                vec[i] += weight * row[i] * s.h
                for j in range(4):
                    mat[i][j] += weight * row[i] * row[j]
        for i in range(4):
            mat[i][i] += 0.02
        sol = solve4(mat, vec)
        if sol is None:
            continue
        cx, cy, hx, hy = sol
        hx = abs(hx)
        hy = abs(hy)
        err = 0.0
        for s in cs:
            pred = (
                cx * s.nx
                + cy * s.ny
                + hx * abs(s.nx * e1[0] + s.ny * e1[1])
                + hy * abs(s.nx * e2[0] + s.ny * e2[1])
            )
            residual = pred - s.h
            ar = abs(residual)
            err += residual * residual if ar < 6 else 36 + (ar - 6) * 6
        err /= max(1.0, weight_sum)
        cand = Fit(cx, cy, hx, hy, float(deg), err, len(cs))
        if best is None or cand.err < best.err:
            best = cand
    return best


def fit_score(fit: Fit | None, expected_short: float, expected_long: float) -> float:
    if fit is None:
        return float("inf")
    short, long = fit.sorted_size
    ratio_penalty = abs((long / max(short, 1e-6)) - (expected_long / expected_short)) * 20.0
    size_penalty = abs(short - expected_short) + abs(long - expected_long)
    return size_penalty + ratio_penalty + fit.err * 0.2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--expected-short-mm", type=float, default=DEFAULT_EXPECTED_SHORT_MM)
    parser.add_argument("--expected-long-mm", type=float, default=DEFAULT_EXPECTED_LONG_MM)
    parser.add_argument("--limit", type=int, default=20)
    args = parser.parse_args()

    meta, frames = parse_log(args.log)
    if not frames:
        print("no raw_frame lines found")
        return 1

    constants = meta.get("constants") or {}
    cal = frames[0].cal or meta.get("current_cal") or {}
    center_d = num(cal.get("centerD"), DEFAULT_CENTER_D_MM)
    max_dist = num(cal.get("maxDist"), 165.0)
    radius = num(cal.get("radius"), 75.0)
    dir_sign = num(cal.get("dir"), -1.0)
    angle_scale = num(cal.get("angleScale"), 1.0)
    sensor_z = num(constants.get("sensor_z_mm"), DEFAULT_SENSOR_Z_MM)
    hfov_deg = num(constants.get("hfov_deg"), DEFAULT_HFOV_DEG)
    vfov_deg = num(constants.get("vfov_deg"), DEFAULT_VFOV_DEG)
    z_floor = num(constants.get("lf_z_floor_mm"), DEFAULT_Z_FLOOR_MM)
    z_ceil = num(constants.get("lf_z_ceil_mm"), DEFAULT_Z_CEIL_MM)
    support_bin_deg = num(constants.get("lf_support_bin_deg"), DEFAULT_SUPPORT_BIN_DEG)

    mappings = [
        "identity",
        "transpose",
        "flip_x",
        "flip_y",
        "rot180",
        "transpose_flip_x",
        "transpose_flip_y",
    ]
    support_modes = ["front_high_mean", "front_max", "front_top3", "old_percentile"]
    robust_modes = ["new", "old"]
    min_dists = sorted({42.0, 45.0, 48.0, num(cal.get("minDist"), 42.0)})

    results = []
    for mapping in mappings:
        for support_mode in support_modes:
            for robust_mode in robust_modes:
                for min_dist in min_dists:
                    supports = build_supports(
                        frames,
                        mapping=mapping,
                        support_mode=support_mode,
                        robust_mode=robust_mode,
                        min_dist=min_dist,
                        max_dist=max_dist,
                        center_d=center_d,
                        sensor_z=sensor_z,
                        hfov_deg=hfov_deg,
                        vfov_deg=vfov_deg,
                        z_floor=z_floor,
                        z_ceil=z_ceil,
                        radius=radius,
                        dir_sign=dir_sign,
                        angle_scale=angle_scale,
                        support_bin_deg=support_bin_deg,
                    )
                    fit = fit_rectangle(supports)
                    score = fit_score(fit, args.expected_short_mm, args.expected_long_mm)
                    results.append((score, mapping, support_mode, robust_mode, min_dist, fit))

    results.sort(key=lambda x: x[0])
    first_angle = frames[0].angle
    last_angle = frames[-1].angle
    print(f"frames={len(frames)} angle={first_angle:.2f}->{last_angle:.2f} expected={args.expected_short_mm:.1f}x{args.expected_long_mm:.1f}mm")
    print(f"centerD={center_d:.1f} maxDist={max_dist:.1f} sensorZ={sensor_z:.1f} hfov={hfov_deg:.1f} vfov={vfov_deg:.1f}")
    print("rank score size_mm ratio supports err mapping support robust minDist center phi")
    for rank, (score, mapping, support_mode, robust_mode, min_dist, fit) in enumerate(results[: args.limit], 1):
        if fit is None:
            print(f"{rank:02d} inf no_fit {mapping} {support_mode} {robust_mode} {min_dist:.0f}")
            continue
        short, long = fit.sorted_size
        ratio = long / max(short, 1e-6)
        print(
            f"{rank:02d} {score:6.2f} {short:5.2f}x{long:5.2f} {ratio:5.3f} "
            f"{fit.supports:3d} {fit.err:5.2f} {mapping:16s} {support_mode:15s} "
            f"{robust_mode:3s} {min_dist:5.1f} ({fit.cx:5.1f},{fit.cy:5.1f}) {fit.phi_deg:5.1f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
