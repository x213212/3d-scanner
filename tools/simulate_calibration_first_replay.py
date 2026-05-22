#!/usr/bin/env python3
"""Replay VL53L7CX raw logs with the firmware calibration-first geometry.

This script intentionally starts from raw_frame distance/status arrays:
1. build an empty-scene background model per angle bin and zone,
2. keep only object readings that are closer than that background,
3. convert foreground support lines into a convex visual hull point cloud.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


ANGLE_BINS = 72
HFOV_DEG = 60.0
VFOV_DEG = 60.0
SENSOR_Z_MM = 61.0
BACKGROUND_VALID_MAX_MM = 300.0
BACKGROUND_MIN_SAMPLES = 3
BACKGROUND_MARGIN_MM = 8.0
SUPPORT_SAMPLE_QUANTILE = 0.35
VISUAL_HULL_RADIUS_MM = 90.0
VISUAL_HULL_EDGE_STEP_MM = 2.0
VISUAL_HULL_Z_STEP_MM = 6.0
MIN_SUPPORT_ZONES = 5
MIN_HULL_SUPPORTS = 12
SHAPE_FIT_MIN_SUPPORTS = 60
ELLIPSE_MIN_CIRCULARITY = 0.80
ELLIPSE_MAX_ASPECT = 3.0
DEFAULT_CENTER_D_MM = 50.0
NO_BG_SUPPORT_QUANTILES = [0.30, 0.40, 0.50, 0.60, 0.70]
NO_BG_DISPLAY_QUANTILE = 0.50
NO_BG_UNION_MIN_SUPPORTS = 60
QUALITY_MAX_SUPPORT_GAP_DEG = 120.0
QUALITY_MIN_ANGULAR_COVERAGE_PCT = 45
QUALITY_MAX_EDGE_RATIO = 0.72
QUALITY_MIN_CENTER_RATIO = 0.12


def read_frames(path: Path) -> list[dict]:
    frames: list[dict] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("raw_frame "):
            frames.append(json.loads(line[len("raw_frame ") :]))
    return frames


def angle_bin(angle: float) -> int:
    return int((((angle % 360.0) + 360.0) % 360.0) / (360.0 / ANGLE_BINS)) % ANGLE_BINS


def build_background(empty_frames: list[dict]) -> list[list[list[float]]]:
    bg = [[[] for _ in range(64)] for _ in range(ANGLE_BINS)]
    for frame in empty_frames:
        b = angle_bin(float(frame.get("server_angle") or 0.0))
        for zone, d in enumerate(frame.get("dist", [])[:64]):
            d = float(d or 0)
            if 0 < d < BACKGROUND_VALID_MAX_MM:
                bg[b][zone].append(d)
    return bg


def background_median(bg: list[list[list[float]]], angle: float, zone: int) -> tuple[float, int] | None:
    b = angle_bin(angle)
    vals: list[float] = []
    for db in (-1, 0, 1):
        vals.extend(bg[(b + db) % ANGLE_BINS][zone])
    if len(vals) < BACKGROUND_MIN_SAMPLES:
        return None
    vals.sort()
    return vals[len(vals) // 2], len(vals)


def zone_row_col(zone: int, mapping: str) -> tuple[int, int]:
    row = zone // 8
    col = zone % 8
    if mapping == "horizontal_cw":
        return 7 - row, col
    if mapping == "horizontal_ccw":
        return row, 7 - col
    if mapping == "sensor":
        return 7 - col, 7 - row
    if mapping == "flip_x":
        return row, 7 - col
    if mapping == "flip_y":
        return 7 - row, col
    if mapping == "rot180":
        return 7 - row, 7 - col
    if mapping == "transpose":
        return col, row
    return row, col


def ray_for_zone(zone: int, mapping: str) -> tuple[float, float, float, int, int]:
    row, col = zone_row_col(zone, mapping)
    yaw = ((col + 0.5) / 8.0 - 0.5) * math.radians(HFOV_DEG)
    pitch = (0.5 - (row + 0.5) / 8.0) * math.radians(VFOV_DEG)
    x = math.sin(yaw) * math.cos(pitch)
    y = -math.cos(yaw) * math.cos(pitch)
    z = math.sin(pitch)
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    return x / n, y / n, z / n, row, col


def clockwise_delta(start: float, end: float) -> float:
    return (end - start + 360.0) % 360.0


def rotate2(x: float, y: float, deg: float) -> tuple[float, float]:
    a = math.radians(deg)
    c = math.cos(a)
    s = math.sin(a)
    return c * x - s * y, s * x + c * y


def foreground_points(
    frame: dict,
    bg: list[list[list[float]]] | None,
    *,
    center_d: float,
    mapping: str,
    distance_model: str,
    min_dist: float,
    max_dist: float,
) -> list[tuple[float, float, float, int, int, int, float]]:
    angle = float(frame.get("server_angle") or 0.0)
    out = []
    for zone, raw_d in enumerate(frame.get("dist", [])[:64]):
        d = float(raw_d or 0)
        if not (min_dist < d < max_dist):
            continue
        if bg is None:
            status = (frame.get("status") or [None] * 64)[zone]
            if status not in (5, 6, 9):
                continue
        else:
            bg_stat = background_median(bg, angle, zone)
            if bg_stat is None:
                continue
            bg_d, _ = bg_stat
            if d >= bg_d - BACKGROUND_MARGIN_MM:
                continue
        dx, dy, dz, row, col = ray_for_zone(zone, mapping)
        scale = d
        if distance_model == "perpendicular":
            scale = d / max(0.12, -dy)
        x = dx * scale
        y = center_d + dy * scale
        z = SENSOR_Z_MM + dz * scale
        if not (8 <= z <= 165 and abs(x) <= 120 and -130 <= y <= 130):
            continue
        out.append((x, y, z, row, col, zone, d))
    return out


def support_from_frame(
    points: list[tuple[float, float, float, int, int, int, float]],
    *,
    no_bg: bool = False,
    no_bg_quantile: float = NO_BG_DISPLAY_QUANTILE,
) -> float | None:
    if len(points) < 2:
        return None
    mid = points if no_bg else [p for p in points if 2 <= p[3] <= 5]
    if len(mid) < 2:
        mid = points
    ys = sorted(p[1] for p in mid)
    if no_bg:
        return ys[max(0, min(len(ys) - 1, round((len(ys) - 1) * no_bg_quantile)))]
    keep = 2 if len(ys) >= 5 else 1
    return sum(ys[-keep:]) / keep


def quantile(values: list[float], q: float) -> float:
    vals = sorted(values)
    return vals[max(0, min(len(vals) - 1, round((len(vals) - 1) * q)))]


def build_supports(
    object_frames: list[dict],
    bg: list[list[list[float]]],
    *,
    center_d: float,
    mapping: str,
    distance_model: str,
    min_dist: float,
    max_dist: float,
    direction: float,
    angle_scale: float,
    min_support_zones: int,
    no_bg_quantile: float = NO_BG_DISPLAY_QUANTILE,
) -> tuple[dict[int, dict], list[tuple[float, float, float]], int, int]:
    supports: dict[int, dict] = {}
    zero_angle: float | None = None
    z_values: list[float] = []
    foreground_total = 0
    foreground_frames = 0
    for frame in object_frames:
        angle = float(frame.get("server_angle") or 0.0)
        pts = foreground_points(frame, bg, center_d=center_d, mapping=mapping, distance_model=distance_model, min_dist=min_dist, max_dist=max_dist)
        foreground_total += len(pts)
        if len(pts) >= min_support_zones:
            foreground_frames += 1
        if len(pts) < min_support_zones:
            continue
        h = support_from_frame(pts, no_bg=bg is None, no_bg_quantile=no_bg_quantile)
        if h is None:
            continue
        if zero_angle is None:
            zero_angle = angle
        fixed_angle = direction * clockwise_delta(zero_angle, angle) * angle_scale
        nx, ny = rotate2(0.0, 1.0, fixed_angle)
        bin_id = round(fixed_angle / 3.0)
        samples = supports.get(bin_id, {}).get("samples", [])
        samples = (samples + [h])[-24:]
        supports[bin_id] = {
            "angle": fixed_angle,
            "nx": nx,
            "ny": ny,
            "h": quantile(samples, SUPPORT_SAMPLE_QUANTILE),
            "count": supports.get(bin_id, {}).get("count", 0) + 1,
            "samples": samples,
        }
        for p in pts:
            if h - p[1] <= 18:
                z_values.append(p[2])
    z0 = max(4.0, min(z_values) - 2.0) if z_values else 18.0
    z1 = min(160.0, max(z_values) + 2.0) if z_values else 135.0
    return supports, [(z0, z1, 0.0)], foreground_total, foreground_frames


def foreground_cell_quality(
    frames: list[dict],
    bg: list[list[list[float]]],
    *,
    center_d: float,
    mapping: str,
    distance_model: str,
    min_dist: float,
    max_dist: float,
) -> dict:
    total = 0
    edge = 0
    center = 0
    cells = [0] * 64
    for frame in frames:
        pts = foreground_points(frame, bg, center_d=center_d, mapping=mapping, distance_model=distance_model, min_dist=min_dist, max_dist=max_dist)
        for _, _, _, row, col, _, _ in pts:
            total += 1
            cells[row * 8 + col] += 1
            if row in (0, 7) or col in (0, 7):
                edge += 1
            if 2 <= row <= 5 and 2 <= col <= 5:
                center += 1
    return {
        "foreground_points": total,
        "active_cells": sum(1 for v in cells if v > 0),
        "edge_ratio": round(edge / total, 3) if total else 0.0,
        "center_ratio": round(center / total, 3) if total else 0.0,
    }


def support_quality(supports: dict[int, dict], cell_quality: dict) -> dict:
    angles = sorted((s["angle"] % 360.0) for s in supports.values() if math.isfinite(s["angle"]))
    largest_gap = 360.0
    if len(angles) >= 2:
        largest_gap = 0.0
        for i, a in enumerate(angles):
            b = angles[0] + 360.0 if i == len(angles) - 1 else angles[i + 1]
            largest_gap = max(largest_gap, b - a)
    coverage = round(max(0.0, min(100.0, (360.0 - largest_gap) * 100.0 / 360.0))) if len(angles) >= 2 else 0
    status = "no-data"
    reason = "no foreground"
    if supports or cell_quality.get("foreground_points", 0):
        status = "partial"
        reason = "insufficient angular support"
        if len(supports) < MIN_HULL_SUPPORTS:
            status = "no-hull"
            reason = "too few support angles"
        elif cell_quality["edge_ratio"] > QUALITY_MAX_EDGE_RATIO and cell_quality["center_ratio"] < QUALITY_MIN_CENTER_RATIO:
            status = "needs-geometry"
            reason = "object is mostly on sensor edge cells"
        elif len(supports) < SHAPE_FIT_MIN_SUPPORTS or coverage < QUALITY_MIN_ANGULAR_COVERAGE_PCT or largest_gap > QUALITY_MAX_SUPPORT_GAP_DEG:
            status = "partial"
            reason = "angle coverage not enough"
        else:
            status = "good"
            reason = "enough angular and zone coverage"
    return {
        "status": status,
        "reason": reason,
        "supports": len(supports),
        "angular_coverage_pct": coverage,
        "largest_gap_deg": round(largest_gap, 1),
        "fit_allowed": status == "good",
        **cell_quality,
    }


def clip_polygon(poly: list[tuple[float, float]], nx: float, ny: float, h: float) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    eps = 1e-9
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        da = nx * a[0] + ny * a[1] - h
        db = nx * b[0] + ny * b[1] - h
        in_a = da <= eps
        in_b = db <= eps
        if in_a and in_b:
            out.append(b)
        elif in_a != in_b:
            t = da / (da - db) if abs(da - db) > eps else 0.0
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            out.append(p)
            if in_b:
                out.append(b)
    return out


def visual_hull(supports: dict[int, dict]) -> list[tuple[float, float]]:
    if len(supports) < MIN_HULL_SUPPORTS:
        return []
    r = VISUAL_HULL_RADIUS_MM
    poly = [(-r, -r), (r, -r), (r, r), (-r, r)]
    for s in sorted(supports.values(), key=lambda v: v["angle"] % 360.0):
        poly = clip_polygon(poly, s["nx"], s["ny"], s["h"])
        if len(poly) < 3:
            return []
    return poly


def polygon_obb_detail(poly: list[tuple[float, float]]) -> dict | None:
    if len(poly) < 3:
        return None
    best = None
    for deg in range(180):
        a = math.radians(deg)
        c = math.cos(a)
        s = math.sin(a)
        xs = [c * x + s * y for x, y in poly]
        ys = [-s * x + c * y for x, y in poly]
        w = max(xs) - min(xs)
        h = max(ys) - min(ys)
        area = w * h
        if best is None or area < best[0]:
            best = (area, w, h, deg, min(xs), max(xs), min(ys), max(ys))
    assert best is not None
    _, w, h, deg, min_x, max_x, min_y, max_y = best
    a = math.radians(deg)
    c = math.cos(a)
    s = math.sin(a)
    corners = []
    for u, v in ((min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)):
        corners.append((c * u - s * v, s * u + c * v))
    cu = (min_x + max_x) / 2
    cv = (min_y + max_y) / 2
    return {
        "short": min(w, h),
        "long": max(w, h),
        "angle": deg,
        "w": w,
        "h": h,
        "cx": c * cu - s * cv,
        "cy": s * cu + c * cv,
        "rx": w / 2,
        "ry": h / 2,
        "phi": math.radians(deg),
        "rect": corners,
        "area": w * h,
    }


def polygon_obb(poly: list[tuple[float, float]]) -> tuple[float, float, int] | None:
    obb = polygon_obb_detail(poly)
    if obb is None:
        return None
    return obb["short"], obb["long"], obb["angle"]


def rectangle_corners(cx: float, cy: float, hx: float, hy: float, phi: float) -> list[tuple[float, float]]:
    c = math.cos(phi)
    s = math.sin(phi)
    out = []
    for u, v in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        out.append((cx + c * u - s * v, cy + s * u + c * v))
    return out


def fit_no_bg_rect_plus_round(supports: dict[int, dict]) -> dict | None:
    if len(supports) < NO_BG_UNION_MIN_SUPPORTS:
        return None
    try:
        import numpy as np
        from scipy.optimize import least_squares
    except Exception:
        return None

    rows = list(supports.values())
    theta = np.array([math.radians(s["angle"]) for s in rows], dtype=float)
    h = np.array([s["h"] for s in rows], dtype=float)
    nx = np.array([s["nx"] for s in rows], dtype=float)
    ny = np.array([s["ny"] for s in rows], dtype=float)
    rough = polygon_obb_detail(visual_hull(supports))
    if rough:
        rough_hx = max(2.0, rough["long"] / 2)
        rough_hy = max(2.0, rough["short"] / 2)
        rough_cx = rough["cx"]
        rough_cy = rough["cy"]
    else:
        rough_hx, rough_hy, rough_cx, rough_cy = 22.0, 16.0, 0.0, 0.0
    starts = [
        [rough_cx, rough_cy, rough_hx, rough_hy, max(8.0, min(rough_hx, rough_hy) * 1.25)],
        [rough_cx, rough_cy, min(rough_hx, rough_hy) * 0.86, max(rough_hx, rough_hy) * 1.02, max(8.0, min(rough_hx, rough_hy) * 1.25)],
        [rough_cx, rough_cy, max(rough_hx, rough_hy) * 1.02, min(rough_hx, rough_hy) * 0.86, max(8.0, min(rough_hx, rough_hy) * 1.25)],
        [0.0, 0.0, 14.0, 22.0, 19.0],
        [0.0, 0.0, 22.0, 14.0, 19.0],
    ]
    best = None
    for deg in range(0, 180, 3):
        phi = math.radians(deg)
        c = math.cos(phi)
        s = math.sin(phi)
        a1 = np.abs(nx * c + ny * s)
        a2 = np.abs(nx * -s + ny * c)

        def residual(p):
            cx, cy, hx, hy, r = p
            trans = cx * nx + cy * ny
            rect = trans + np.abs(hx) * a1 + np.abs(hy) * a2
            round_clutter = trans + np.abs(r)
            rr = np.maximum(rect, round_clutter) - h
            return np.where(np.abs(rr) < 5.0, rr, np.sign(rr) * (5.0 + np.sqrt(np.maximum(0.0, np.abs(rr) - 5.0))))

        for start in starts:
            sol = least_squares(
                residual,
                start,
                bounds=([-50, -50, 2, 2, 0], [50, 50, 80, 80, 80]),
                max_nfev=350,
            )
            err = float(np.mean(residual(sol.x) ** 2))
            cx, cy, hx, hy, r = sol.x
            hx = abs(float(hx))
            hy = abs(float(hy))
            r = abs(float(r))
            score = err + r * r * 0.0005
            if best is None or score < best["score"]:
                best = {
                    "score": score,
                    "err": err,
                    "cx": float(cx),
                    "cy": float(cy),
                    "hx": hx,
                    "hy": hy,
                    "r": r,
                    "phi": phi,
                    "angle": deg,
                    "short": min(hx * 2, hy * 2),
                    "long": max(hx * 2, hy * 2),
                    "rect": rectangle_corners(float(cx), float(cy), hx, hy, phi),
                }
    return best


def polygon_area(poly: list[tuple[float, float]]) -> float:
    area = 0.0
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        area += a[0] * b[1] - b[0] * a[1]
    return abs(area) * 0.5


def polygon_perimeter(poly: list[tuple[float, float]]) -> float:
    return sum(math.dist(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))


def polygon_circularity(poly: list[tuple[float, float]]) -> float:
    perim = polygon_perimeter(poly)
    return 0.0 if perim <= 0 else 4.0 * math.pi * polygon_area(poly) / (perim * perim)


def should_ellipse_fit(poly: list[tuple[float, float]], obb: dict | None) -> bool:
    if len(poly) < 8 or obb is None or obb["short"] <= 0:
        return False
    return obb["long"] / obb["short"] <= ELLIPSE_MAX_ASPECT and polygon_circularity(poly) >= ELLIPSE_MIN_CIRCULARITY


def hull_point_cloud(poly: list[tuple[float, float]], z0: float, z1: float) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    if len(poly) < 3:
        return points
    z_steps = max(2, math.ceil(max(1.0, z1 - z0) / VISUAL_HULL_Z_STEP_MM))
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        edge_steps = max(1, math.ceil(math.dist(a, b) / VISUAL_HULL_EDGE_STEP_MM))
        for e in range(edge_steps + 1):
            t = e / edge_steps
            x = a[0] + (b[0] - a[0]) * t
            y = a[1] + (b[1] - a[1]) * t
            for zi in range(z_steps + 1):
                z = z0 + (z1 - z0) * zi / z_steps
                points.append((x, y, z))
    return points


def ellipse_point_cloud(obb: dict, z0: float, z1: float) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    z_steps = max(2, math.ceil(max(1.0, z1 - z0) / VISUAL_HULL_Z_STEP_MM))
    segs = max(36, math.ceil(math.pi * (abs(obb["rx"]) + abs(obb["ry"])) / VISUAL_HULL_EDGE_STEP_MM))
    c = math.cos(obb["phi"])
    s = math.sin(obb["phi"])
    for i in range(segs):
        t = i * math.tau / segs
        u = math.cos(t) * obb["rx"]
        v = math.sin(t) * obb["ry"]
        x = obb["cx"] + c * u - s * v
        y = obb["cy"] + s * u + c * v
        for zi in range(z_steps + 1):
            z = z0 + (z1 - z0) * zi / z_steps
            points.append((x, y, z))
    return points


def ellipse_outline(obb: dict, segments: int = 96) -> list[tuple[float, float]]:
    c = math.cos(obb["phi"])
    s = math.sin(obb["phi"])
    out = []
    for i in range(segments):
        t = i * math.tau / segments
        u = math.cos(t) * obb["rx"]
        v = math.sin(t) * obb["ry"]
        out.append((obb["cx"] + c * u - s * v, obb["cy"] + s * u + c * v))
    return out


def draw_image(
    out_path: Path,
    empty_poly: list[tuple[float, float]],
    box_poly: list[tuple[float, float]],
    fit_poly: list[tuple[float, float]],
    cloud: list[tuple[float, float, float]],
    obb: tuple[float, float, int] | None,
    *,
    empty_fg: int,
    box_fg: int,
    box_frames: int,
    center_d: float,
    mapping: str,
    distance_model: str,
    shape: str,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(14, 5), dpi=160)
    ax0 = fig.add_subplot(1, 3, 1)
    ax1 = fig.add_subplot(1, 3, 2)
    ax2 = fig.add_subplot(1, 3, 3, projection="3d")

    ax0.set_title(f"empty replay residual={empty_fg}, no stable hull")
    ax0.set_aspect("equal", adjustable="box")
    ax0.set_xlim(-60, 60)
    ax0.set_ylim(-60, 60)
    ax0.grid(True, alpha=0.25)
    if empty_poly:
        xs, ys = zip(*(empty_poly + [empty_poly[0]]))
        ax0.plot(xs, ys, color="tab:red")

    ax1.set_title(f"visual hull + {shape}, center={center_d:g}mm, {distance_model}")
    ax1.set_aspect("equal", adjustable="box")
    ax1.set_xlim(-45, 45)
    ax1.set_ylim(-45, 45)
    ax1.grid(True, alpha=0.25)
    if box_poly:
        xs, ys = zip(*(box_poly + [box_poly[0]]))
        ax1.plot(xs, ys, color="tab:blue", linewidth=1.5, alpha=0.55, label="raw visual hull")
        ax1.scatter(xs[:-1], ys[:-1], s=12, color="tab:blue")
    if fit_poly:
        xs, ys = zip(*(fit_poly + [fit_poly[0]]))
        ax1.plot(xs, ys, color="tab:orange", linewidth=2.4, label=shape)
    ax1.add_patch(plt.Rectangle((-21.5, -14.0), 43.0, 28.0, fill=False, linestyle="--", edgecolor="0.45", label="43x28mm scale"))
    if obb:
        ax1.text(0.02, 0.98, f"OBB {obb[0]:.1f} x {obb[1]:.1f} mm\nsupport frames {box_frames}", transform=ax1.transAxes, va="top")
    ax1.legend(loc="lower right", fontsize=8)

    ax2.set_title("3D surface point cloud from geometry")
    if cloud:
        xs, ys, zs = zip(*cloud)
        ax2.scatter(xs, ys, zs, s=3, c=zs, cmap="viridis", alpha=0.9)
    ax2.set_xlabel("x mm")
    ax2.set_ylabel("y mm")
    ax2.set_zlabel("z mm")
    ax2.set_box_aspect((1, 1, 0.8))
    ax2.view_init(elev=24, azim=-52)

    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def write_ply(path: Path, cloud: list[tuple[float, float, float]], label: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("comment generated by tools/simulate_calibration_first_replay.py\n")
        f.write(f"comment source {label}\n")
        f.write(f"element vertex {len(cloud)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for x, y, z in cloud:
            f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--empty", type=Path, default=Path("vl53l7cx_raw_scan.jsonl"))
    parser.add_argument("--object", type=Path, default=Path("vl53l7cx_raw_scan-2.jsonl"))
    parser.add_argument("--output", type=Path, default=Path("vl53l7cx_calibration_first_visual_hull_sim.png"))
    parser.add_argument("--center-d", type=float, default=DEFAULT_CENTER_D_MM)
    parser.add_argument("--mapping", default="horizontal_cw", choices=["horizontal_cw", "horizontal_ccw", "sensor", "exported", "flip_x", "flip_y", "rot180", "transpose"])
    parser.add_argument("--distance-model", default="perpendicular", choices=["perpendicular", "radial"])
    parser.add_argument("--no-bg", action="store_true", help="Use status-valid raw hits directly instead of empty-scan background subtraction.")
    parser.add_argument("--ply-output", type=Path, help="Optional ASCII PLY point-cloud model output.")
    parser.add_argument("--min-dist", type=float, default=20.0)
    parser.add_argument("--max-dist", type=float, default=165.0)
    parser.add_argument("--min-support-zones", type=int, default=MIN_SUPPORT_ZONES)
    args = parser.parse_args()

    empty_frames = [] if args.no_bg else read_frames(args.empty)
    object_frames = read_frames(args.object)
    bg = None if args.no_bg else build_background(empty_frames)

    empty_supports, empty_z, empty_fg, empty_frames_fg = build_supports(
        empty_frames,
        bg,
        center_d=args.center_d,
        mapping=args.mapping,
        distance_model=args.distance_model,
        min_dist=args.min_dist,
        max_dist=args.max_dist,
        direction=-1.0,
        angle_scale=1.0,
        min_support_zones=args.min_support_zones,
    )
    box_supports, box_z, box_fg, box_frames_fg = build_supports(
        object_frames,
        bg,
        center_d=args.center_d,
        mapping=args.mapping,
        distance_model=args.distance_model,
        min_dist=args.min_dist,
        max_dist=args.max_dist,
        direction=-1.0,
        angle_scale=1.0,
        min_support_zones=args.min_support_zones,
    )
    no_bg_candidates = []
    best_candidate = None
    if args.no_bg:
        for q in NO_BG_SUPPORT_QUANTILES:
            cand_supports, cand_z, cand_fg, cand_frames_fg = build_supports(
                object_frames,
                bg,
                center_d=args.center_d,
                mapping=args.mapping,
                distance_model=args.distance_model,
                min_dist=args.min_dist,
                max_dist=args.max_dist,
                direction=-1.0,
                angle_scale=1.0,
                min_support_zones=args.min_support_zones,
                no_bg_quantile=q,
            )
            cand_fit = fit_no_bg_rect_plus_round(cand_supports)
            if cand_fit:
                item = {
                    "q": q,
                    "supports": len(cand_supports),
                    "fit": cand_fit,
                    "supports_data": cand_supports,
                    "z": cand_z,
                    "fg": cand_fg,
                    "frames_fg": cand_frames_fg,
                }
                no_bg_candidates.append(item)
                if best_candidate is None or cand_fit["score"] < best_candidate["fit"]["score"]:
                    best_candidate = item
        if best_candidate:
            box_supports = best_candidate["supports_data"]
            box_z = best_candidate["z"]
            box_fg = best_candidate["fg"]
            box_frames_fg = best_candidate["frames_fg"]
    empty_poly = visual_hull(empty_supports)
    box_poly = visual_hull(box_supports)
    z0, z1, _ = box_z[0]
    obb_detail = polygon_obb_detail(box_poly)
    quality = support_quality(
        box_supports,
        foreground_cell_quality(
            object_frames,
            bg,
            center_d=args.center_d,
            mapping=args.mapping,
            distance_model=args.distance_model,
            min_dist=args.min_dist,
            max_dist=args.max_dist,
        ),
    )
    if args.no_bg:
        quality["status"] = "no-bg-envelope"
        quality["reason"] = "estimated from raw hits with rectangle-plus-round-clutter model"
        quality["background_ready"] = False
        quality["fit_allowed"] = False
    no_bg_fit = best_candidate["fit"] if args.no_bg and best_candidate else None
    enough_supports = bool(quality["fit_allowed"])
    circle_like = bool(enough_supports and obb_detail and should_ellipse_fit(box_poly, obb_detail) and obb_detail["long"] / max(obb_detail["short"], 1e-9) <= 1.25)
    if circle_like:
        fit_poly = ellipse_outline(obb_detail)
        cloud = ellipse_point_cloud(obb_detail, z0, z1)
        shape = "circle/ellipse fit"
    elif args.no_bg and no_bg_fit:
        fit_poly = no_bg_fit["rect"]
        cloud = hull_point_cloud(fit_poly, z0, z1)
        q_text = "" if best_candidate is None else f" q={best_candidate['q']:.2f}"
        shape = f"no-bg rect+round estimate {no_bg_fit['short']:.1f}x{no_bg_fit['long']:.1f}mm round={no_bg_fit['r']:.1f}mm{q_text}"
    elif not enough_supports:
        fit_poly = []
        cloud = hull_point_cloud(box_poly, z0, z1)
        shape = "no-bg envelope estimate" if args.no_bg else "partial hull (insufficient support coverage)"
    else:
        fit_poly = obb_detail["rect"] if obb_detail else []
        cloud_poly = fit_poly or box_poly
        cloud = hull_point_cloud(cloud_poly, z0, z1)
        shape = "box OBB fit" if fit_poly else "raw hull"
    obb = None if obb_detail is None else (obb_detail["short"], obb_detail["long"], obb_detail["angle"])

    draw_image(
        args.output,
        empty_poly,
        box_poly,
        fit_poly,
        cloud,
        obb,
        empty_fg=empty_fg,
        box_fg=box_fg,
        box_frames=box_frames_fg,
        center_d=args.center_d,
        mapping=args.mapping,
        distance_model=args.distance_model,
        shape=shape,
    )
    if args.ply_output:
        write_ply(args.ply_output, cloud, shape)
    size = "none" if obb is None else f"{obb[0]:.2f}x{obb[1]:.2f}mm angle={obb[2]}"
    if no_bg_fit:
        quality["no_bg_rect_plus_round"] = {
            "short": round(no_bg_fit["short"], 2),
            "long": round(no_bg_fit["long"], 2),
            "round_radius": round(no_bg_fit["r"], 2),
            "angle": no_bg_fit["angle"],
            "err": round(no_bg_fit["err"], 3),
            "support_q": best_candidate["q"] if best_candidate else None,
        }
        quality["no_bg_candidates"] = [
            {
                "q": c["q"],
                "short": round(c["fit"]["short"], 2),
                "long": round(c["fit"]["long"], 2),
                "round_radius": round(c["fit"]["r"], 2),
                "err": round(c["fit"]["err"], 3),
            }
            for c in no_bg_candidates
        ]
    print(f"empty foreground raw={empty_fg} support_frames={empty_frames_fg} hull_vertices={len(empty_poly)}")
    print(f"box foreground raw={box_fg} support_frames={box_frames_fg} supports={len(box_supports)} hull_vertices={len(box_poly)} raw_hull_obb={size}")
    print(f"quality={json.dumps(quality, ensure_ascii=False)}")
    model_out = "" if not args.ply_output else f" ply={args.ply_output}"
    print(f"cloud points={len(cloud)} source={shape} distance_model={args.distance_model} z={z0:.1f}..{z1:.1f} output={args.output}{model_out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
