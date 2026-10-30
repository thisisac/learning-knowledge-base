# -*- coding: utf-8 -*-
"""
Laser point-cloud homework pipeline.

This script processes one LAS file end-to-end:
1. load LAS points without laspy;
2. remove obvious height outliers;
3. extract ground points with two methods:
   - rasterized progressive TIN filtering;
   - cloth simulation filtering;
4. write ground-point LAS files;
5. build DEM GeoTIFF files;
6. render result screenshots and a concise PPTX report.

The implementation intentionally avoids heavyweight point-cloud/GIS packages so
that the project is easy to run in PyCharm. Only numpy is required.
"""

from __future__ import annotations

import math
import struct
import time
import zipfile
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.sax.saxutils import escape

import numpy as np


# ---------------------------------------------------------------------------
# User-editable configuration
# ---------------------------------------------------------------------------

SELECTED_LAS_NAME = "20251126150027848.las"
OUTPUT_DIR_NAME = "outputs"

CHUNK_POINTS = 700_000
MAX_SCREENSHOT_POINTS = 520_000

PTIN_INITIAL_CELL = 4.0
PTIN_STEPS = (
    # (cell_size, height_threshold)
    (2.0, 0.70),
    (1.0, 0.50),
    (0.50, 0.38),
)
PTIN_FINAL_ABOVE = 0.46
PTIN_FINAL_BELOW = 0.30

CSF_CELL = 0.60
CSF_ITERATIONS = 120
CSF_RIGIDNESS = 0.33
CSF_GRAVITY = 0.055
CSF_FINAL_ABOVE = 0.55
CSF_FINAL_BELOW = 0.35

DEM_RESOLUTION = 0.25
NODATA_VALUE = -9999.0


# ---------------------------------------------------------------------------
# LAS handling
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LasHeader:
    path: Path
    header_bytes: bytes
    header_size: int
    point_offset: int
    point_format: int
    point_record_length: int
    point_count: int
    scale: tuple[float, float, float]
    offset: tuple[float, float, float]
    bounds: tuple[float, float, float, float, float, float]
    version_major: int
    version_minor: int

    @property
    def xmin(self) -> float:
        return self.bounds[0]

    @property
    def xmax(self) -> float:
        return self.bounds[1]

    @property
    def ymin(self) -> float:
        return self.bounds[2]

    @property
    def ymax(self) -> float:
        return self.bounds[3]

    @property
    def zmin(self) -> float:
        return self.bounds[4]

    @property
    def zmax(self) -> float:
        return self.bounds[5]


def read_las_header(path: Path) -> LasHeader:
    with path.open("rb") as f:
        initial = f.read(375)

    if initial[:4] != b"LASF":
        raise ValueError(f"{path} is not a LAS file.")

    version_major = initial[24]
    version_minor = initial[25]
    header_size = struct.unpack_from("<H", initial, 94)[0]
    point_offset = struct.unpack_from("<I", initial, 96)[0]
    raw_format = initial[104]
    point_format = raw_format & 0b0011_1111
    point_record_length = struct.unpack_from("<H", initial, 105)[0]
    point_count = struct.unpack_from("<I", initial, 107)[0]

    if version_major == 1 and version_minor >= 4 and len(initial) >= 255:
        extended_count = struct.unpack_from("<Q", initial, 247)[0]
        if extended_count:
            point_count = int(extended_count)

    scale = struct.unpack_from("<ddd", initial, 131)
    offset = struct.unpack_from("<ddd", initial, 155)
    max_x, min_x, max_y, min_y, max_z, min_z = struct.unpack_from(
        "<dddddd", initial, 179
    )

    with path.open("rb") as f:
        header_bytes = f.read(point_offset)

    return LasHeader(
        path=path,
        header_bytes=header_bytes,
        header_size=header_size,
        point_offset=point_offset,
        point_format=point_format,
        point_record_length=point_record_length,
        point_count=point_count,
        scale=scale,
        offset=offset,
        bounds=(min_x, max_x, min_y, max_y, min_z, max_z),
        version_major=version_major,
        version_minor=version_minor,
    )


def point_dtype(record_length: int) -> np.dtype:
    return np.dtype(
        {
            "names": ["x", "y", "z", "return_flags", "classification"],
            "formats": ["<i4", "<i4", "<i4", "u1", "u1"],
            "offsets": [0, 4, 8, 14, 15],
            "itemsize": record_length,
        }
    )


def iter_las_records(header: LasHeader, chunk_points: int = CHUNK_POINTS):
    dtype = point_dtype(header.point_record_length)
    with header.path.open("rb") as f:
        f.seek(header.point_offset)
        remaining = header.point_count
        while remaining:
            n = min(chunk_points, remaining)
            raw = f.read(n * header.point_record_length)
            if not raw:
                break
            actual = len(raw) // header.point_record_length
            if actual == 0:
                break
            raw = raw[: actual * header.point_record_length]
            yield raw, np.frombuffer(raw, dtype=dtype, count=actual)
            remaining -= actual


def load_points(header: LasHeader):
    print(f"Loading LAS points: {header.path.name}")
    n = header.point_count
    x = np.empty(n, dtype=np.float32)
    y = np.empty(n, dtype=np.float32)
    z = np.empty(n, dtype=np.float32)
    return_flags = np.empty(n, dtype=np.uint8)

    sx, sy, sz = header.scale
    ox, oy, oz = header.offset
    cursor = 0
    started = time.time()

    for _, rec in iter_las_records(header):
        k = len(rec)
        x[cursor : cursor + k] = rec["x"].astype(np.float64) * sx + ox
        y[cursor : cursor + k] = rec["y"].astype(np.float64) * sy + oy
        z[cursor : cursor + k] = rec["z"].astype(np.float64) * sz + oz
        return_flags[cursor : cursor + k] = rec["return_flags"]
        cursor += k
        if cursor and cursor % 2_800_000 < CHUNK_POINTS:
            print(f"  loaded {cursor:,}/{n:,} points")

    if cursor != n:
        x = x[:cursor]
        y = y[:cursor]
        z = z[:cursor]
        return_flags = return_flags[:cursor]

    print(f"  done in {time.time() - started:.1f}s")
    return x, y, z, return_flags


def update_header_for_subset(
    header: LasHeader,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    return_flags: np.ndarray,
    mask: np.ndarray,
) -> bytes:
    out = bytearray(header.header_bytes)
    count = int(mask.sum())
    if count == 0:
        raise ValueError("No ground points were selected; cannot write LAS.")

    struct.pack_into("<I", out, 107, count)
    returns = return_flags[mask] & 0b0000_0111
    return_counts = np.bincount(returns, minlength=8)
    for i in range(5):
        struct.pack_into("<I", out, 111 + i * 4, int(return_counts[i + 1]))

    sx = x[mask]
    sy = y[mask]
    sz = z[mask]
    struct.pack_into(
        "<dddddd",
        out,
        179,
        float(sx.max()),
        float(sx.min()),
        float(sy.max()),
        float(sy.min()),
        float(sz.max()),
        float(sz.min()),
    )
    return bytes(out)


def write_ground_las(
    header: LasHeader,
    out_path: Path,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    return_flags: np.ndarray,
    mask: np.ndarray,
):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    new_header = update_header_for_subset(header, x, y, z, return_flags, mask)

    print(f"Writing ground LAS: {out_path.name} ({int(mask.sum()):,} points)")
    with out_path.open("wb") as out:
        out.write(new_header)
        cursor = 0
        for raw, rec in iter_las_records(header):
            k = len(rec)
            chunk_mask = mask[cursor : cursor + k]
            if chunk_mask.any():
                raw_records = np.frombuffer(
                    raw, dtype=np.dtype((np.void, header.point_record_length))
                )
                selected = bytearray(raw_records[chunk_mask].tobytes())
                # LAS 1.2 classification byte: keep high flag bits, set class 2.
                for pos in range(15, len(selected), header.point_record_length):
                    selected[pos] = (selected[pos] & 0b1110_0000) | 2
                out.write(selected)
            cursor += k


# ---------------------------------------------------------------------------
# Point-cloud filtering
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class GridSpec:
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    cell: float
    rows: int
    cols: int


def make_grid_spec(
    x: np.ndarray, y: np.ndarray, cell: float, margin: float = 0.001
) -> GridSpec:
    xmin = float(np.nanmin(x)) - margin
    xmax = float(np.nanmax(x)) + margin
    ymin = float(np.nanmin(y)) - margin
    ymax = float(np.nanmax(y)) + margin
    cols = int(math.ceil((xmax - xmin) / cell)) + 1
    rows = int(math.ceil((ymax - ymin) / cell)) + 1
    return GridSpec(xmin, xmax, ymin, ymax, cell, rows, cols)


def grid_indices(x: np.ndarray, y: np.ndarray, spec: GridSpec):
    cols = np.floor((x - spec.xmin) / spec.cell).astype(np.int64)
    rows = np.floor((y - spec.ymin) / spec.cell).astype(np.int64)
    np.clip(cols, 0, spec.cols - 1, out=cols)
    np.clip(rows, 0, spec.rows - 1, out=rows)
    return rows, cols


def minimum_grid(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    mask: np.ndarray,
    spec: GridSpec,
) -> np.ndarray:
    rows, cols = grid_indices(x[mask], y[mask], spec)
    flat_index = rows * spec.cols + cols
    grid = np.full(spec.rows * spec.cols, np.inf, dtype=np.float32)
    np.minimum.at(grid, flat_index, z[mask].astype(np.float32))
    grid = grid.reshape(spec.rows, spec.cols)
    grid[~np.isfinite(grid)] = np.nan
    return grid


def mean_grid(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    mask: np.ndarray,
    spec: GridSpec,
) -> np.ndarray:
    rows, cols = grid_indices(x[mask], y[mask], spec)
    flat_index = rows * spec.cols + cols
    sums = np.zeros(spec.rows * spec.cols, dtype=np.float64)
    counts = np.zeros(spec.rows * spec.cols, dtype=np.uint32)
    np.add.at(sums, flat_index, z[mask].astype(np.float64))
    np.add.at(counts, flat_index, 1)
    dem = np.full(spec.rows * spec.cols, np.nan, dtype=np.float32)
    valid = counts > 0
    dem[valid] = (sums[valid] / counts[valid]).astype(np.float32)
    return dem.reshape(spec.rows, spec.cols)


def fill_nan_grid(grid: np.ndarray, fallback: np.ndarray | None = None) -> np.ndarray:
    filled = grid.astype(np.float32, copy=True)
    if fallback is not None:
        empty = ~np.isfinite(filled)
        filled[empty] = fallback[empty]

    if np.isfinite(filled).any():
        default_value = float(np.nanmedian(filled))
    else:
        default_value = 0.0

    for _ in range(160):
        empty = ~np.isfinite(filled)
        if not empty.any():
            break
        pad = np.pad(filled, 1, mode="constant", constant_values=np.nan)
        total = np.zeros_like(filled, dtype=np.float32)
        count = np.zeros_like(filled, dtype=np.uint8)
        for dr in range(3):
            for dc in range(3):
                if dr == 1 and dc == 1:
                    continue
                nb = pad[dr : dr + filled.shape[0], dc : dc + filled.shape[1]]
                ok = np.isfinite(nb)
                total[ok] += nb[ok]
                count[ok] += 1
        can_fill = empty & (count > 0)
        if not can_fill.any():
            break
        filled[can_fill] = total[can_fill] / count[can_fill]

    filled[~np.isfinite(filled)] = default_value
    return filled


def smooth_grid(grid: np.ndarray, iterations: int = 2, strength: float = 0.45):
    out = grid.astype(np.float32, copy=True)
    for _ in range(iterations):
        pad = np.pad(out, 1, mode="edge")
        avg = (
            pad[:-2, :-2]
            + pad[:-2, 1:-1]
            + pad[:-2, 2:]
            + pad[1:-1, :-2]
            + pad[1:-1, 1:-1] * 2.0
            + pad[1:-1, 2:]
            + pad[2:, :-2]
            + pad[2:, 1:-1]
            + pad[2:, 2:]
        ) / 10.0
        out = (1.0 - strength) * out + strength * avg
    return out.astype(np.float32)


def resample_surface(surface: np.ndarray, from_spec: GridSpec, to_spec: GridSpec):
    row_coords = to_spec.ymin + (np.arange(to_spec.rows, dtype=np.float32) + 0.5) * to_spec.cell
    col_coords = to_spec.xmin + (np.arange(to_spec.cols, dtype=np.float32) + 0.5) * to_spec.cell
    yy, xx = np.meshgrid(row_coords, col_coords, indexing="ij")
    return surface_values(surface, from_spec, xx.ravel(), yy.ravel()).reshape(
        to_spec.rows, to_spec.cols
    )


def surface_values(
    surface: np.ndarray, spec: GridSpec, x: np.ndarray, y: np.ndarray
) -> np.ndarray:
    cols = np.floor((x - spec.xmin) / spec.cell).astype(np.int64)
    rows = np.floor((y - spec.ymin) / spec.cell).astype(np.int64)
    np.clip(cols, 0, spec.cols - 1, out=cols)
    np.clip(rows, 0, spec.rows - 1, out=rows)
    return surface[rows, cols]


def build_progressive_tin_surface(
    x: np.ndarray, y: np.ndarray, z: np.ndarray, denoise_mask: np.ndarray
):
    print("Building rasterized progressive TIN ground surface")
    spec = make_grid_spec(x, y, PTIN_INITIAL_CELL)
    seed = minimum_grid(x, y, z, denoise_mask, spec)
    surface = smooth_grid(fill_nan_grid(seed), iterations=2, strength=0.40)

    for cell, threshold in PTIN_STEPS:
        next_spec = make_grid_spec(x, y, cell)
        base = surface_values(surface, spec, x, y)
        accepted = denoise_mask & (z >= base - 0.55) & (z <= base + threshold)
        grid = minimum_grid(x, y, z, accepted, next_spec)
        fallback = resample_surface(surface, spec, next_spec)
        surface = smooth_grid(
            fill_nan_grid(grid, fallback=fallback), iterations=2, strength=0.34
        )
        spec = next_spec
        print(
            f"  PTIN step cell={cell:.2f}m threshold={threshold:.2f}m "
            f"accepted={int(accepted.sum()):,}"
        )

    return surface, spec


def build_cloth_surface(
    x: np.ndarray, y: np.ndarray, z: np.ndarray, denoise_mask: np.ndarray
):
    print("Building cloth simulation filtering ground surface")
    spec = make_grid_spec(x, y, CSF_CELL)
    min_surface = minimum_grid(x, y, z, denoise_mask, spec)
    obstacle = -smooth_grid(fill_nan_grid(min_surface), iterations=1, strength=0.20)
    cloth = np.full_like(obstacle, float(np.nanmax(obstacle) + 2.0), dtype=np.float32)

    for i in range(CSF_ITERATIONS):
        pad = np.pad(cloth, 1, mode="edge")
        neighbor_avg = (
            pad[:-2, 1:-1] + pad[2:, 1:-1] + pad[1:-1, :-2] + pad[1:-1, 2:]
        ) * 0.25
        cloth = cloth + CSF_RIGIDNESS * (neighbor_avg - cloth)
        cloth = cloth - CSF_GRAVITY
        cloth = np.maximum(cloth, obstacle)
        if (i + 1) % 40 == 0:
            print(f"  CSF iteration {i + 1}/{CSF_ITERATIONS}")

    ground_surface = -smooth_grid(cloth, iterations=2, strength=0.25)
    return ground_surface, spec


def classify_ground(
    method_name: str,
    surface: np.ndarray,
    spec: GridSpec,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    denoise_mask: np.ndarray,
    above: float,
    below: float,
) -> np.ndarray:
    base = surface_values(surface, spec, x, y)
    dz = z - base
    mask = denoise_mask & (dz <= above) & (dz >= -below)
    ratio = 100.0 * float(mask.sum()) / float(len(z))
    print(f"{method_name} ground points: {int(mask.sum()):,} ({ratio:.2f}%)")
    return mask


# ---------------------------------------------------------------------------
# DEM, GeoTIFF, PNG rendering
# ---------------------------------------------------------------------------


def build_dem(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    mask: np.ndarray,
    resolution: float = DEM_RESOLUTION,
) -> tuple[np.ndarray, GridSpec]:
    spec = make_grid_spec(x, y, resolution)
    dem_bottom_up = mean_grid(x, y, z, mask, spec)
    dem_bottom_up = smooth_grid(fill_nan_grid(dem_bottom_up), iterations=1, strength=0.18)
    dem_top_down = dem_bottom_up[::-1, :]
    return dem_top_down.astype(np.float32), spec


def write_geotiff_float32(
    path: Path,
    raster: np.ndarray,
    spec: GridSpec,
    nodata: float = NODATA_VALUE,
):
    path.parent.mkdir(parents=True, exist_ok=True)
    data = np.asarray(raster, dtype="<f4")
    data = np.where(np.isfinite(data), data, np.float32(nodata)).astype("<f4")
    height, width = data.shape

    tags: list[tuple[int, int, int, object]] = [
        (256, 4, 1, width),  # ImageWidth
        (257, 4, 1, height),  # ImageLength
        (258, 3, 1, 32),  # BitsPerSample
        (259, 3, 1, 1),  # Compression = none
        (262, 3, 1, 1),  # PhotometricInterpretation = BlackIsZero
        (273, 4, 1, 0),  # StripOffsets, filled after extra values
        (277, 3, 1, 1),  # SamplesPerPixel
        (278, 4, 1, height),  # RowsPerStrip
        (279, 4, 1, int(width * height * 4)),  # StripByteCounts
        (284, 3, 1, 1),  # PlanarConfiguration
        (339, 3, 1, 3),  # SampleFormat = IEEE floating point
        (33550, 12, 3, (spec.cell, spec.cell, 0.0)),  # ModelPixelScaleTag
        (
            33922,
            12,
            6,
            (0.0, 0.0, 0.0, spec.xmin, spec.ymax, 0.0),
        ),  # ModelTiepointTag
        (
            34735,
            3,
            12,
            (1, 1, 0, 2, 1024, 0, 1, 1, 1025, 0, 1, 1),
        ),  # GeoKeyDirectoryTag: projected/local, pixel-is-area
        (42113, 2, len(str(nodata)) + 1, f"{nodata}\0"),  # GDAL_NODATA
    ]

    type_sizes = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 11: 4, 12: 8}
    n_tags = len(tags)
    ifd_size = 2 + n_tags * 12 + 4
    extra_offset = 8 + ifd_size
    extra_chunks: list[bytes] = []

    def pack_value(tag_type: int, count: int, value: object):
        nonlocal extra_offset
        size = type_sizes[tag_type] * count
        if tag_type == 2:
            raw = str(value).encode("ascii")
        elif tag_type == 3:
            values = value if isinstance(value, tuple) else (value,)
            raw = struct.pack("<" + "H" * count, *values)
        elif tag_type == 4:
            values = value if isinstance(value, tuple) else (value,)
            raw = struct.pack("<" + "I" * count, *values)
        elif tag_type == 12:
            values = value if isinstance(value, tuple) else (value,)
            raw = struct.pack("<" + "d" * count, *values)
        else:
            raise ValueError(f"Unsupported TIFF tag type: {tag_type}")

        if size <= 4:
            return raw.ljust(4, b"\0")
        offset = extra_offset
        extra_chunks.append(raw)
        extra_offset += len(raw)
        if extra_offset % 2:
            extra_chunks.append(b"\0")
            extra_offset += 1
        return struct.pack("<I", offset)

    # Compute image data offset after all extra tag data.
    extra_offset_probe = extra_offset
    for tag, tag_type, count, value in tags:
        if tag == 273:
            continue
        size = type_sizes[tag_type] * count
        if tag_type == 2:
            size = len(str(value).encode("ascii"))
        if size > 4:
            extra_offset_probe += size + (size % 2)
    image_offset = extra_offset_probe

    encoded_entries = []
    extra_chunks = []
    extra_offset = 8 + ifd_size
    for tag, tag_type, count, value in tags:
        if tag == 273:
            value_bytes = struct.pack("<I", image_offset)
        else:
            value_bytes = pack_value(tag_type, count, value)
        encoded_entries.append(struct.pack("<HHI4s", tag, tag_type, count, value_bytes))

    with path.open("wb") as f:
        f.write(b"II")
        f.write(struct.pack("<H", 42))
        f.write(struct.pack("<I", 8))
        f.write(struct.pack("<H", n_tags))
        for entry in encoded_entries:
            f.write(entry)
        f.write(struct.pack("<I", 0))
        for chunk in extra_chunks:
            f.write(chunk)
        f.write(data.tobytes(order="C"))


def write_png(path: Path, rgb: np.ndarray):
    path.parent.mkdir(parents=True, exist_ok=True)
    arr = np.asarray(rgb, dtype=np.uint8)
    if arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError("PNG writer expects an RGB uint8 array.")
    height, width = arr.shape[:2]

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    raw_rows = b"".join(b"\x00" + arr[row].tobytes() for row in range(height))
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(
        b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    )
    png += chunk(b"IDAT", zlib.compress(raw_rows, level=6))
    png += chunk(b"IEND", b"")
    path.write_bytes(png)


def colorize(values: np.ndarray, vmin: float, vmax: float) -> np.ndarray:
    palette = np.array(
        [
            [42, 72, 88],
            [35, 132, 122],
            [126, 176, 92],
            [226, 190, 92],
            [198, 112, 70],
            [238, 238, 228],
        ],
        dtype=np.float32,
    )
    t = (values.astype(np.float32) - vmin) / max(vmax - vmin, 1e-6)
    t = np.clip(t, 0.0, 1.0)
    scaled = t * (len(palette) - 1)
    lo = np.floor(scaled).astype(np.int32)
    hi = np.clip(lo + 1, 0, len(palette) - 1)
    frac = (scaled - lo)[..., None]
    rgb = palette[lo] * (1.0 - frac) + palette[hi] * frac
    return np.clip(rgb, 0, 255).astype(np.uint8)


def sample_indices(n: int, max_points: int = MAX_SCREENSHOT_POINTS) -> np.ndarray:
    if n <= max_points:
        return np.arange(n, dtype=np.int64)
    return np.linspace(0, n - 1, max_points, dtype=np.int64)


def render_point_map(
    path: Path,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    bounds: tuple[float, float, float, float],
    width: int = 1400,
    height: int = 900,
    background: np.ndarray | None = None,
):
    canvas = np.full((height, width, 3), 250, dtype=np.uint8)
    xmin, xmax, ymin, ymax = bounds
    margin = 34
    scale = min((width - 2 * margin) / (xmax - xmin), (height - 2 * margin) / (ymax - ymin))
    px = (margin + (x - xmin) * scale).astype(np.int32)
    py = (height - margin - (y - ymin) * scale).astype(np.int32)
    valid = (px >= 0) & (px < width) & (py >= 0) & (py < height)

    if background is not None:
        bx, by = background
        bpx = (margin + (bx - xmin) * scale).astype(np.int32)
        bpy = (height - margin - (by - ymin) * scale).astype(np.int32)
        bvalid = (bpx >= 0) & (bpx < width) & (bpy >= 0) & (bpy < height)
        canvas[bpy[bvalid], bpx[bvalid]] = np.array([214, 218, 220], dtype=np.uint8)

    if valid.any():
        lo, hi = np.percentile(z[valid], [2, 98])
        colors = colorize(z[valid], float(lo), float(hi))
        canvas[py[valid], px[valid]] = colors

    write_png(path, canvas)


def render_dem(path: Path, dem: np.ndarray):
    data = dem.astype(np.float32)
    valid = np.isfinite(data)
    canvas = np.full((*data.shape, 3), 248, dtype=np.uint8)
    if valid.any():
        lo, hi = np.percentile(data[valid], [2, 98])
        rgb = colorize(data, float(lo), float(hi)).astype(np.float32)
        gy, gx = np.gradient(np.where(valid, data, np.nanmedian(data[valid])))
        slope = np.sqrt(gx * gx + gy * gy)
        shade = 1.0 - np.clip(slope / (np.percentile(slope, 95) + 1e-6), 0, 1) * 0.35
        rgb = rgb * shade[..., None]
        canvas[valid] = np.clip(rgb[valid], 0, 255).astype(np.uint8)
    write_png(path, canvas)


TINY_FONT = {
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    "C": ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "G": ["01111", "10000", "10000", "10011", "10001", "10001", "01110"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "J": ["00111", "00010", "00010", "00010", "00010", "10010", "01100"],
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "Q": ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
    "W": ["10001", "10001", "10001", "10101", "10101", "10101", "01010"],
    "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    "Z": ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "10000", "11110", "00001", "00001", "11110"],
    "6": ["01110", "10000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00001", "01110"],
    ".": ["00000", "00000", "00000", "00000", "00000", "01100", "01100"],
    ",": ["00000", "00000", "00000", "00000", "01100", "01100", "01000"],
    ":": ["00000", "01100", "01100", "00000", "01100", "01100", "00000"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    "+": ["00000", "00100", "00100", "11111", "00100", "00100", "00000"],
    "/": ["00001", "00010", "00010", "00100", "01000", "01000", "10000"],
    "%": ["11001", "11010", "00010", "00100", "01000", "01011", "10011"],
    "(": ["00010", "00100", "01000", "01000", "01000", "00100", "00010"],
    ")": ["01000", "00100", "00010", "00010", "00010", "00100", "01000"],
    "=": ["00000", "11111", "00000", "00000", "11111", "00000", "00000"],
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
}


def draw_text(
    canvas: np.ndarray,
    text: str,
    x: int,
    y: int,
    color: tuple[int, int, int] = (25, 32, 44),
    scale: int = 2,
):
    cx = int(x)
    cy = int(y)
    for ch in text.upper():
        bitmap = TINY_FONT.get(ch, TINY_FONT[" "])
        for r, row in enumerate(bitmap):
            for c, value in enumerate(row):
                if value == "1":
                    y0 = cy + r * scale
                    y1 = min(canvas.shape[0], y0 + scale)
                    x0 = cx + c * scale
                    x1 = min(canvas.shape[1], x0 + scale)
                    if x1 > 0 and y1 > 0 and x0 < canvas.shape[1] and y0 < canvas.shape[0]:
                        canvas[max(0, y0) : y1, max(0, x0) : x1] = color
        cx += 6 * scale


def draw_rect(
    canvas: np.ndarray,
    x0: int,
    y0: int,
    x1: int,
    y1: int,
    color: tuple[int, int, int],
    fill: bool = False,
):
    h, w = canvas.shape[:2]
    x0, x1 = sorted((max(0, int(x0)), min(w - 1, int(x1))))
    y0, y1 = sorted((max(0, int(y0)), min(h - 1, int(y1))))
    if fill:
        canvas[y0 : y1 + 1, x0 : x1 + 1] = color
    else:
        canvas[y0, x0 : x1 + 1] = color
        canvas[y1, x0 : x1 + 1] = color
        canvas[y0 : y1 + 1, x0] = color
        canvas[y0 : y1 + 1, x1] = color


def draw_line(
    canvas: np.ndarray,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    color: tuple[int, int, int],
    width: int = 1,
):
    length = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
    xs = np.linspace(x0, x1, length).astype(np.int32)
    ys = np.linspace(y0, y1, length).astype(np.int32)
    half = max(0, width // 2)
    h, w = canvas.shape[:2]
    for dx in range(-half, half + 1):
        for dy in range(-half, half + 1):
            xx = xs + dx
            yy = ys + dy
            valid = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
            canvas[yy[valid], xx[valid]] = color


def draw_polyline(
    canvas: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    color: tuple[int, int, int],
    width: int = 1,
):
    if len(xs) < 2:
        return
    for i in range(len(xs) - 1):
        draw_line(canvas, xs[i], ys[i], xs[i + 1], ys[i + 1], color, width)


def chart_canvas(title: str, width: int = 1200, height: int = 760):
    canvas = np.full((height, width, 3), 248, dtype=np.uint8)
    draw_text(canvas, title, 36, 26, (17, 24, 39), 3)
    return canvas


def chart_box(canvas: np.ndarray):
    left, top, right, bottom = 86, 96, canvas.shape[1] - 42, canvas.shape[0] - 76
    draw_rect(canvas, left, top, right, bottom, (82, 96, 111), False)
    for frac in (0.25, 0.50, 0.75):
        y = bottom - int((bottom - top) * frac)
        draw_line(canvas, left, y, right, y, (224, 229, 235), 1)
    return left, top, right, bottom


def draw_histogram_lines(
    canvas: np.ndarray,
    bins: np.ndarray,
    series: list[tuple[str, np.ndarray, tuple[int, int, int]]],
    log_scale: bool = True,
):
    left, top, right, bottom = chart_box(canvas)
    max_y = 1.0
    processed = []
    for _, counts, color in series:
        values = np.log1p(counts.astype(np.float64)) if log_scale else counts.astype(np.float64)
        processed.append((values, color))
        max_y = max(max_y, float(values.max()))
    x_positions = np.linspace(left, right, len(bins) - 1)
    for values, color in processed:
        y_positions = bottom - (values / max_y) * (bottom - top)
        draw_polyline(canvas, x_positions, y_positions, color, 3)
    legend_x = left + 10
    for i, (label, _, color) in enumerate(series):
        y = top + 20 + i * 34
        draw_rect(canvas, legend_x, y, legend_x + 28, y + 16, color, True)
        draw_text(canvas, label, legend_x + 42, y - 2, (31, 41, 55), 2)
    draw_text(canvas, "ELEVATION M", (left + right) // 2 - 80, bottom + 30, (51, 65, 85), 2)
    draw_text(canvas, "LOG COUNT", left - 62, top - 32, (51, 65, 85), 2)


def render_height_histogram(
    path: Path,
    z: np.ndarray,
    ptin_mask: np.ndarray,
    csf_mask: np.ndarray,
):
    canvas = chart_canvas("HEIGHT HISTOGRAM")
    lo, hi = np.percentile(z, [0.2, 99.8])
    bins = np.linspace(float(lo), float(hi), 90)
    raw_counts, _ = np.histogram(z, bins=bins)
    ptin_counts, _ = np.histogram(z[ptin_mask], bins=bins)
    csf_counts, _ = np.histogram(z[csf_mask], bins=bins)
    draw_histogram_lines(
        canvas,
        bins,
        [
            ("RAW", raw_counts, (71, 85, 105)),
            ("PTIN", ptin_counts, (13, 148, 136)),
            ("CSF", csf_counts, (217, 119, 6)),
        ],
    )
    write_png(path, canvas)


def render_denoise_effect(path: Path, z: np.ndarray, denoise_mask: np.ndarray):
    canvas = chart_canvas("DENOISE EFFECT")
    lo, hi = np.percentile(z, [0.02, 99.98])
    bins = np.linspace(float(lo), float(hi), 90)
    kept_counts, _ = np.histogram(z[denoise_mask], bins=bins)
    removed_counts, _ = np.histogram(z[~denoise_mask], bins=bins)
    draw_histogram_lines(
        canvas,
        bins,
        [
            ("KEPT", kept_counts, (22, 163, 74)),
            ("REMOVED", removed_counts, (220, 38, 38)),
        ],
    )
    removed = int((~denoise_mask).sum())
    draw_text(canvas, f"REMOVED {removed:,}".replace(",", ""), 92, 690, (127, 29, 29), 2)
    write_png(path, canvas)


def render_filter_comparison_bar(
    path: Path,
    total_count: int,
    ptin_count: int,
    csf_count: int,
):
    canvas = chart_canvas("FILTER COMPARISON")
    left, top, right, bottom = chart_box(canvas)
    counts = np.array([ptin_count, csf_count], dtype=np.float64)
    labels = ["PTIN", "CSF"]
    colors = [(13, 148, 136), (217, 119, 6)]
    max_count = max(total_count, int(counts.max()))
    bar_width = 180
    xs = [left + 260, left + 620]
    for x0, count, label, color in zip(xs, counts, labels, colors):
        h = int((count / max_count) * (bottom - top))
        draw_rect(canvas, x0, bottom - h, x0 + bar_width, bottom, color, True)
        draw_rect(canvas, x0, bottom - h, x0 + bar_width, bottom, (30, 41, 59), False)
        draw_text(canvas, label, x0 + 44, bottom + 26, (31, 41, 55), 3)
        percent = 100.0 * count / max(total_count, 1)
        draw_text(canvas, f"{percent:.1f}%", x0 + 30, bottom - h - 34, (31, 41, 55), 2)
    draw_text(canvas, "GROUND POINT RATIO", left + 360, bottom + 64, (51, 65, 85), 2)
    write_png(path, canvas)


def density_palette(values: np.ndarray) -> np.ndarray:
    palette = np.array(
        [
            [248, 250, 252],
            [209, 233, 246],
            [103, 169, 207],
            [27, 120, 180],
            [18, 87, 138],
            [76, 29, 149],
        ],
        dtype=np.float32,
    )
    t = np.clip(values.astype(np.float32), 0.0, 1.0)
    scaled = t * (len(palette) - 1)
    lo = np.floor(scaled).astype(np.int32)
    hi = np.clip(lo + 1, 0, len(palette) - 1)
    frac = (scaled - lo)[..., None]
    return np.clip(palette[lo] * (1.0 - frac) + palette[hi] * frac, 0, 255).astype(np.uint8)


def render_point_density_heatmap(path: Path, x: np.ndarray, y: np.ndarray):
    title_h = 76
    grid_w, grid_h = 1100, 720
    canvas = np.full((title_h + grid_h + 30, grid_w + 80, 3), 248, dtype=np.uint8)
    draw_text(canvas, "POINT DENSITY HEATMAP", 36, 26, (17, 24, 39), 3)
    cols, rows = 500, 330
    xmin, xmax = float(x.min()), float(x.max())
    ymin, ymax = float(y.min()), float(y.max())
    xi = np.floor((x - xmin) / max(xmax - xmin, 1e-6) * (cols - 1)).astype(np.int64)
    yi = np.floor((y - ymin) / max(ymax - ymin, 1e-6) * (rows - 1)).astype(np.int64)
    counts = np.zeros(rows * cols, dtype=np.uint32)
    np.add.at(counts, yi * cols + xi, 1)
    counts = counts.reshape(rows, cols)[::-1, :]
    values = np.log1p(counts.astype(np.float32))
    values = values / max(float(values.max()), 1e-6)
    heat = density_palette(values)
    sy = math.ceil(grid_h / rows)
    sx = math.ceil(grid_w / cols)
    heat = np.repeat(np.repeat(heat, sy, axis=0), sx, axis=1)[:grid_h, :grid_w]
    canvas[title_h : title_h + grid_h, 40 : 40 + grid_w] = heat
    draw_rect(canvas, 40, title_h, 40 + grid_w - 1, title_h + grid_h - 1, (51, 65, 85), False)
    draw_text(canvas, "LOW", 46, title_h + grid_h + 8, (51, 65, 85), 2)
    draw_text(canvas, "HIGH", 40 + grid_w - 78, title_h + grid_h + 8, (51, 65, 85), 2)
    write_png(path, canvas)


def render_class_overlap_map(
    path: Path,
    x: np.ndarray,
    y: np.ndarray,
    ptin_mask: np.ndarray,
    csf_mask: np.ndarray,
    bounds: tuple[float, float, float, float],
):
    idx = sample_indices(len(x), MAX_SCREENSHOT_POINTS)
    xs = x[idx]
    ys = y[idx]
    ptin = ptin_mask[idx]
    csf = csf_mask[idx]
    width, height = 1400, 900
    canvas = np.full((height, width, 3), 250, dtype=np.uint8)
    xmin, xmax, ymin, ymax = bounds
    margin = 42
    scale = min((width - 2 * margin) / (xmax - xmin), (height - 2 * margin) / (ymax - ymin))
    px = (margin + (xs - xmin) * scale).astype(np.int32)
    py = (height - margin - (ys - ymin) * scale).astype(np.int32)
    valid = (px >= 0) & (px < width) & (py >= 0) & (py < height)
    colors = np.full((len(idx), 3), [210, 214, 219], dtype=np.uint8)
    both = ptin & csf
    colors[both] = [22, 163, 74]
    colors[ptin & ~csf] = [37, 99, 235]
    colors[~ptin & csf] = [217, 119, 6]
    canvas[py[valid], px[valid]] = colors[valid]
    draw_text(canvas, "PTIN VS CSF OVERLAP", 42, 30, (17, 24, 39), 3)
    legend = [
        ("BOTH", (22, 163, 74)),
        ("PTIN ONLY", (37, 99, 235)),
        ("CSF ONLY", (217, 119, 6)),
        ("NON GROUND", (148, 163, 184)),
    ]
    for i, (label, color) in enumerate(legend):
        y0 = 92 + i * 34
        draw_rect(canvas, 46, y0, 74, y0 + 16, color, True)
        draw_text(canvas, label, 86, y0 - 2, (31, 41, 55), 2)
    write_png(path, canvas)


def render_dem_hillshade(path: Path, dem: np.ndarray):
    data = dem.astype(np.float32)
    valid = np.isfinite(data)
    z = np.where(valid, data, np.nanmedian(data[valid]))
    gy, gx = np.gradient(z)
    azimuth = math.radians(315.0)
    altitude = math.radians(45.0)
    slope = np.pi / 2.0 - np.arctan(np.sqrt(gx * gx + gy * gy))
    aspect = np.arctan2(-gx, gy)
    shade = np.sin(altitude) * np.sin(slope) + np.cos(altitude) * np.cos(slope) * np.cos(azimuth - aspect)
    shade = np.clip((shade + 1.0) * 0.5, 0.0, 1.0)
    rgb = (shade[..., None] * np.array([235, 238, 241]) + (1 - shade[..., None]) * np.array([42, 51, 65])).astype(np.uint8)
    write_png(path, rgb)


def render_dem_contour(path: Path, dem: np.ndarray):
    data = dem.astype(np.float32)
    valid = np.isfinite(data)
    lo, hi = np.percentile(data[valid], [2, 98])
    canvas = colorize(data, float(lo), float(hi))
    levels = np.linspace(float(lo), float(hi), 14)
    tolerance = max((hi - lo) / 230.0, 0.01)
    for level in levels:
        near = valid & (np.abs(data - level) <= tolerance)
        canvas[near] = np.array([18, 24, 38], dtype=np.uint8)
    draw_text(canvas, "DEM CONTOUR", 18, 18, (15, 23, 42), 2)
    write_png(path, canvas)


def diverging_color(values: np.ndarray, limit: float) -> np.ndarray:
    t = np.clip(values / max(limit, 1e-6), -1.0, 1.0)
    rgb = np.zeros((*values.shape, 3), dtype=np.float32)
    pos = t >= 0
    neg = ~pos
    pos_t = t[pos][:, None]
    neg_t = t[neg][:, None]
    rgb[pos] = np.array([244, 244, 245]) * (1 - pos_t) + np.array([185, 28, 28]) * pos_t
    rgb[neg] = np.array([244, 244, 245]) * (1 + neg_t) + np.array([37, 99, 235]) * (-neg_t)
    return np.clip(rgb, 0, 255).astype(np.uint8)


def render_dem_difference(path: Path, dem_ptin: np.ndarray, dem_csf: np.ndarray):
    diff = dem_csf.astype(np.float32) - dem_ptin.astype(np.float32)
    valid = np.isfinite(diff)
    limit = float(np.percentile(np.abs(diff[valid]), 98)) if valid.any() else 1.0
    canvas = diverging_color(np.where(valid, diff, 0.0), limit)
    draw_text(canvas, "CSF - PTIN DEM", 18, 18, (15, 23, 42), 2)
    draw_text(canvas, "BLUE LOW  RED HIGH", 18, canvas.shape[0] - 32, (15, 23, 42), 2)
    write_png(path, canvas)


def render_terrain_profile(path: Path, dem_ptin: np.ndarray, dem_csf: np.ndarray, spec: GridSpec):
    canvas = chart_canvas("TERRAIN PROFILE")
    left, top, right, bottom = chart_box(canvas)
    row = dem_ptin.shape[0] // 2
    ptin = dem_ptin[row, :].astype(np.float32)
    csf = dem_csf[row, :].astype(np.float32)
    valid = np.isfinite(ptin) & np.isfinite(csf)
    ptin = ptin[valid]
    csf = csf[valid]
    dist = np.arange(len(ptin), dtype=np.float32) * spec.cell
    lo = float(min(ptin.min(), csf.min()))
    hi = float(max(ptin.max(), csf.max()))
    xs = left + (dist / max(float(dist.max()), 1e-6)) * (right - left)
    y_ptin = bottom - ((ptin - lo) / max(hi - lo, 1e-6)) * (bottom - top)
    y_csf = bottom - ((csf - lo) / max(hi - lo, 1e-6)) * (bottom - top)
    draw_polyline(canvas, xs, y_ptin, (13, 148, 136), 3)
    draw_polyline(canvas, xs, y_csf, (217, 119, 6), 3)
    draw_rect(canvas, left + 16, top + 20, left + 48, top + 36, (13, 148, 136), True)
    draw_text(canvas, "PTIN", left + 62, top + 18, (31, 41, 55), 2)
    draw_rect(canvas, left + 150, top + 20, left + 182, top + 36, (217, 119, 6), True)
    draw_text(canvas, "CSF", left + 196, top + 18, (31, 41, 55), 2)
    draw_text(canvas, "DISTANCE M", (left + right) // 2 - 70, bottom + 30, (51, 65, 85), 2)
    draw_text(canvas, "ELEVATION M", left - 62, top - 32, (51, 65, 85), 2)
    write_png(path, canvas)


def render_dem_3d_view(path: Path, dem: np.ndarray):
    data = dem.astype(np.float32)
    valid = np.isfinite(data)
    z = np.where(valid, data, np.nanmedian(data[valid]))
    h, w = z.shape
    step = max(2, int(max(h, w) / 120))
    rows = np.arange(0, h, step)
    cols = np.arange(0, w, step)
    zz = z[np.ix_(rows, cols)]
    lo, hi = float(np.percentile(zz, 2)), float(np.percentile(zz, 98))
    canvas = np.full((900, 1300, 3), 248, dtype=np.uint8)
    draw_text(canvas, "DEM 3D VIEW", 38, 28, (17, 24, 39), 3)
    sx = 7.2
    sy = 3.8
    vz = 42.0 / max(hi - lo, 1e-6)
    x_offset = 210
    y_offset = 230
    for r in range(zz.shape[0] - 1, -1, -1):
        xs = []
        ys = []
        colors = []
        for c in range(zz.shape[1]):
            xp = x_offset + c * sx + r * sx * 0.52
            yp = y_offset + r * sy - (zz[r, c] - lo) * vz
            xs.append(xp)
            ys.append(yp)
            colors.append(zz[r, c])
        color = tuple(int(v) for v in colorize(np.array([np.mean(colors)]), lo, hi)[0])
        draw_polyline(canvas, np.array(xs), np.array(ys), color, 2)
    write_png(path, canvas)


def render_analysis_figures(
    output_dir: Path,
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    denoise_mask: np.ndarray,
    ptin_mask: np.ndarray,
    csf_mask: np.ndarray,
    dem_ptin: np.ndarray,
    dem_csf: np.ndarray,
    dem_spec: GridSpec,
    bounds_2d: tuple[float, float, float, float],
) -> dict[str, float | int]:
    print("Rendering additional report analysis figures")
    render_height_histogram(output_dir / "height_histogram.png", z, ptin_mask, csf_mask)
    render_point_density_heatmap(output_dir / "point_density_heatmap.png", x, y)
    render_denoise_effect(output_dir / "denoise_effect.png", z, denoise_mask)
    render_filter_comparison_bar(
        output_dir / "filter_comparison_bar.png",
        len(z),
        int(ptin_mask.sum()),
        int(csf_mask.sum()),
    )
    render_class_overlap_map(
        output_dir / "ptin_vs_csf_overlap.png", x, y, ptin_mask, csf_mask, bounds_2d
    )
    render_dem_hillshade(output_dir / "dem_hillshade.png", dem_csf)
    render_dem_contour(output_dir / "dem_contour.png", dem_csf)
    render_dem_difference(output_dir / "dem_difference.png", dem_ptin, dem_csf)
    render_terrain_profile(output_dir / "terrain_profile.png", dem_ptin, dem_csf, dem_spec)
    render_dem_3d_view(output_dir / "dem_3d_view.png", dem_csf)

    both = int((ptin_mask & csf_mask).sum())
    ptin_only = int((ptin_mask & ~csf_mask).sum())
    csf_only = int((~ptin_mask & csf_mask).sum())
    diff = dem_csf.astype(np.float32) - dem_ptin.astype(np.float32)
    valid = np.isfinite(diff)
    return {
        "both_ground_count": both,
        "ptin_only_count": ptin_only,
        "csf_only_count": csf_only,
        "dem_diff_mean": float(np.nanmean(diff[valid])) if valid.any() else 0.0,
        "dem_diff_std": float(np.nanstd(diff[valid])) if valid.any() else 0.0,
        "dem_diff_abs_p95": float(np.nanpercentile(np.abs(diff[valid]), 95)) if valid.any() else 0.0,
    }


# ---------------------------------------------------------------------------
# Report assets
# ---------------------------------------------------------------------------


def write_flowchart_svg(path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    boxes = [
        ("Start: read LAS and header", 60, 30),
        ("Height outlier removal", 60, 115),
        ("Method A: progressive TIN filtering", 60, 200),
        ("Method B: cloth simulation filtering", 410, 200),
        ("Write ground-point LAS files", 235, 300),
        ("Interpolate ground points to DEM", 235, 385),
        ("Export GeoTIFF, PNGs and PPTX report", 235, 470),
    ]
    rects = []
    for text, x, y in boxes:
        rects.append(
            f'<rect x="{x}" y="{y}" width="290" height="50" rx="8" '
            'fill="#f7fafc" stroke="#334155" stroke-width="1.5"/>'
            f'<text x="{x + 145}" y="{y + 30}" text-anchor="middle" '
            'font-size="14" font-family="Arial, Microsoft YaHei" '
            f'fill="#111827">{escape(text)}</text>'
        )
    arrows = [
        (205, 80, 205, 115),
        (205, 165, 205, 200),
        (350, 225, 410, 225),
        (205, 250, 320, 300),
        (555, 250, 380, 300),
        (380, 350, 380, 385),
        (380, 435, 380, 470),
    ]
    lines = []
    for x1, y1, x2, y2 in arrows:
        lines.append(
            f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
            'stroke="#475569" stroke-width="1.6" marker-end="url(#arrow)"/>'
        )
    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="760" height="550" viewBox="0 0 760 550">
  <defs>
    <marker id="arrow" markerWidth="10" markerHeight="10" refX="9" refY="3"
            orient="auto" markerUnits="strokeWidth">
      <path d="M0,0 L0,6 L9,3 z" fill="#475569"/>
    </marker>
  </defs>
  <rect width="760" height="550" fill="white"/>
  {''.join(lines)}
  {''.join(rects)}
</svg>
'''
    path.write_text(svg, encoding="utf-8")


def write_report_notes(path: Path, context: dict[str, object]):
    overlap_ratio = 100.0 * context["both_ground_count"] / max(context["point_count"], 1)
    ptin_ratio = 100.0 * context["ptin_count"] / max(context["point_count"], 1)
    csf_ratio = 100.0 * context["csf_count"] / max(context["point_count"], 1)
    lines = [
        "# 激光三维建模作业处理说明",
        "",
        f"- 处理文件：`{context['las_name']}`",
        f"- 原始点数：{context['point_count']:,}",
        f"- 去噪后参与滤波点数：{context['denoised_count']:,}",
        f"- 渐进式 TIN 地面点数：{context['ptin_count']:,}，占原始点云 {ptin_ratio:.2f}%",
        f"- 布料滤波地面点数：{context['csf_count']:,}，占原始点云 {csf_ratio:.2f}%",
        f"- 两种方法共同判为地面点：{context['both_ground_count']:,}，占原始点云 {overlap_ratio:.2f}%",
        f"- DEM 差值（CSF - PTIN）均值：{context['dem_diff_mean']:.3f} m，标准差：{context['dem_diff_std']:.3f} m，95% 绝对差值：{context['dem_diff_abs_p95']:.3f} m",
        f"- DEM 分辨率：{DEM_RESOLUTION} m",
        "",
        "## 地面滤波原理",
        "",
        "渐进式三角网滤波先从大网格最低点构建稀疏地面种子面，随后逐步减小网格尺度，并用高度阈值把接近当前地面的点加入地面集合。"
        "本程序采用栅格化的渐进式 TIN 思路实现，避免依赖 SciPy/Open3D，同时保留“由粗到细、逐步加密地面面片”的核心逻辑。",
        "",
        "布料滤波将点云高程取反，在反向点云上方放置一张规则布料；布料在重力和邻域弹性约束下下降，碰到反向地形后停止。"
        "最终将布料面再取反，得到近似地表面，并按点到布料面的高差提取地面点。",
        "",
        "## 输出成果",
        "",
        "- `ground_ptin.las`：渐进式 TIN 方法地面点。",
        "- `ground_csf.las`：布料滤波方法地面点。",
        "- `dem_ptin.tif`、`dem_csf.tif`：地面点插值得到的 GeoTIFF DEM。",
        "- `raw_point_cloud.png`、`ground_ptin.png`、`ground_csf.png`、`dem_csf.png`：基础报告截图素材。",
        "- `homework_report.pptx`：可直接打开的简明实验报告 PPT。",
        "",
        "## 新增可视化图及报告写法",
        "",
        "- `height_histogram.png`：展示原始点云、PTIN 地面点、CSF 地面点的高程分布。报告中可说明：地面点主要集中在低高程区间，滤波后高程分布更贴近堤防地表。",
        "- `point_density_heatmap.png`：展示点云平面采样密度。报告中可说明：密度较高区域对应扫描覆盖充分区域，低密度区可能影响 DEM 插值稳定性。",
        "- `denoise_effect.png`：展示去噪前后高程分布。报告中可说明：仅剔除了极端高程异常点，主体地形信息被保留。",
        "- `filter_comparison_bar.png`：对比两种算法提取的地面点数量和比例。报告中可说明：CSF 提取地面点略多，PTIN 判别相对保守。",
        "- `ptin_vs_csf_overlap.png`：展示两种滤波结果的重叠和差异。报告中可说明：共同地面点代表稳定地表区域，单方法区域体现算法阈值和模型假设差异。",
        "- `dem_hillshade.png`：DEM 阴影地形图。报告中可用于表达地形起伏、坡面和堤防线形。",
        "- `dem_contour.png`：DEM 等高线图。报告中可用于分析高程连续性和局部坡度变化。",
        "- `dem_difference.png`：CSF DEM 与 PTIN DEM 的差值图。报告中可说明：蓝色表示 CSF 低于 PTIN，红色表示 CSF 高于 PTIN，差异集中区通常对应滤波边界或地形突变区。",
        "- `terrain_profile.png`：中心横剖面图。报告中可用于展示两种 DEM 在剖面上的一致性和局部差异。",
        "- `dem_3d_view.png`：DEM 伪三维透视图。报告中可作为地形级三维成果展示图。",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def pptx_xml_textbox(
    shape_id: int,
    name: str,
    x: int,
    y: int,
    w: int,
    h: int,
    text: str,
    font_size: int = 2400,
    bold: bool = False,
) -> str:
    runs = []
    for line in text.split("\n"):
        runs.append(
            '<a:r><a:rPr lang="zh-CN" '
            f'sz="{font_size}"{" b=\"1\"" if bold else ""}/>'
            f"<a:t>{escape(line)}</a:t></a:r>"
        )
        runs.append("<a:br/>")
    if runs:
        runs.pop()
    return f'''
<p:sp>
  <p:nvSpPr><p:cNvPr id="{shape_id}" name="{escape(name)}"/><p:cNvSpPr txBox="1"/><p:nvPr/></p:nvSpPr>
  <p:spPr><a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{w}" cy="{h}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom><a:noFill/></p:spPr>
  <p:txBody><a:bodyPr wrap="square"/><a:lstStyle/><a:p>{''.join(runs)}</a:p></p:txBody>
</p:sp>'''


def pptx_xml_picture(
    shape_id: int,
    name: str,
    rid: str,
    x: int,
    y: int,
    w: int,
    h: int,
) -> str:
    return f'''
<p:pic>
  <p:nvPicPr><p:cNvPr id="{shape_id}" name="{escape(name)}"/><p:cNvPicPr/><p:nvPr/></p:nvPicPr>
  <p:blipFill><a:blip r:embed="{rid}"/><a:stretch><a:fillRect/></a:stretch></p:blipFill>
  <p:spPr><a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{w}" cy="{h}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></p:spPr>
</p:pic>'''


def make_slide_xml(shapes: Iterable[str]) -> str:
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sld xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
       xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
       xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:cSld><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
    {''.join(shapes)}
  </p:spTree></p:cSld>
  <p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>
</p:sld>'''


def _create_basic_report_pptx(out_path: Path, output_dir: Path, context: dict[str, object]):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    images = {
        "raw": output_dir / "raw_point_cloud.png",
        "ptin": output_dir / "ground_ptin.png",
        "csf": output_dir / "ground_csf.png",
        "dem": output_dir / "dem_csf.png",
    }
    media_names = {key: f"image{i}.png" for i, key in enumerate(images, start=1)}

    slide_w = 13_333_500
    slide_h = 7_500_000
    title = pptx_xml_textbox(
        2,
        "Title",
        560_000,
        360_000,
        12_200_000,
        650_000,
        "激光点云地面滤波与 DEM 构建",
        3400,
        True,
    )
    subtitle = pptx_xml_textbox(
        3,
        "Subtitle",
        700_000,
        1_250_000,
        11_500_000,
        3_900_000,
        (
            f"数据文件：{context['las_name']}\n"
            f"原始点数：{context['point_count']:,}\n"
            f"方法：渐进式 TIN 滤波、布料滤波\n"
            f"成果：地面点 LAS、GeoTIFF DEM、效果截图"
        ),
        2200,
    )
    slide1 = make_slide_xml([title, subtitle])

    slide2_text = (
        "技术流程\n"
        "1. 读取 LAS 头信息和点记录，得到坐标、回波与分类字段。\n"
        "2. 使用全局稳健高程阈值剔除明显噪声点。\n"
        "3. 渐进式 TIN：最低点种子面 -> 逐级加密 -> 高差阈值判别地面点。\n"
        "4. 布料滤波：高程取反 -> 布料下降碰撞 -> 反算地表面。\n"
        "5. 对地面点栅格平均、空洞填补并输出 DEM GeoTIFF。"
    )
    slide2 = make_slide_xml(
        [
            pptx_xml_textbox(2, "Flow", 600_000, 540_000, 12_000_000, 6_000_000, slide2_text, 2100),
        ]
    )

    slide3 = make_slide_xml(
        [
            pptx_xml_textbox(2, "Title", 500_000, 250_000, 12_200_000, 450_000, "原始点云俯视效果", 2600, True),
            pptx_xml_picture(3, "Raw", "rId1", 850_000, 850_000, 11_650_000, 5_950_000),
        ]
    )

    slide4 = make_slide_xml(
        [
            pptx_xml_textbox(2, "Title", 500_000, 250_000, 12_200_000, 450_000, "两种地面滤波效果对比", 2600, True),
            pptx_xml_picture(3, "PTIN", "rId1", 650_000, 950_000, 5_950_000, 4_800_000),
            pptx_xml_picture(4, "CSF", "rId2", 6_750_000, 950_000, 5_950_000, 4_800_000),
            pptx_xml_textbox(5, "Label1", 1_050_000, 5_880_000, 5_200_000, 420_000, "渐进式 TIN 地面点", 1700, True),
            pptx_xml_textbox(6, "Label2", 7_250_000, 5_880_000, 5_200_000, 420_000, "布料滤波地面点", 1700, True),
        ]
    )

    slide5 = make_slide_xml(
        [
            pptx_xml_textbox(2, "Title", 500_000, 250_000, 12_200_000, 450_000, "DEM 成果", 2600, True),
            pptx_xml_picture(3, "DEM", "rId1", 900_000, 900_000, 7_800_000, 5_700_000),
            pptx_xml_textbox(
                4,
                "Summary",
                9_050_000,
                1_150_000,
                3_700_000,
                4_800_000,
                (
                    f"DEM 分辨率：{DEM_RESOLUTION} m\n"
                    f"PTIN 地面点：{context['ptin_count']:,}\n"
                    f"CSF 地面点：{context['csf_count']:,}\n"
                    "GeoTIFF 使用 LAS 本地坐标写入像元比例尺和定位标签。"
                ),
                1850,
            ),
        ]
    )

    slide_rels = {
        1: "",
        2: "",
        3: '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="../media/image1.png"/>',
        4: (
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="../media/image2.png"/>'
            '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="../media/image3.png"/>'
        ),
        5: '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="../media/image4.png"/>',
    }
    slides = [slide1, slide2, slide3, slide4, slide5]

    presentation_rels = [
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="slideMasters/slideMaster1.xml"/>'
    ]
    for i in range(1, len(slides) + 1):
        presentation_rels.append(
            f'<Relationship Id="rId{i + 1}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide" Target="slides/slide{i}.xml"/>'
        )

    slide_id_list = "\n".join(
        f'<p:sldId id="{255 + i}" r:id="rId{i + 1}"/>' for i in range(1, len(slides) + 1)
    )
    overrides = "\n".join(
        f'<Override PartName="/ppt/slides/slide{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slide+xml"/>'
        for i in range(1, len(slides) + 1)
    )

    content_types = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Default Extension="png" ContentType="image/png"/>
  <Override PartName="/ppt/presentation.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.presentation.main+xml"/>
  <Override PartName="/ppt/slideMasters/slideMaster1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideMaster+xml"/>
  <Override PartName="/ppt/slideLayouts/slideLayout1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideLayout+xml"/>
  <Override PartName="/ppt/theme/theme1.xml" ContentType="application/vnd.openxmlformats-officedocument.theme+xml"/>
  {overrides}
</Types>'''
    root_rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="ppt/presentation.xml"/>
</Relationships>'''
    presentation = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:presentation xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
                xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
                xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:sldMasterIdLst><p:sldMasterId id="2147483648" r:id="rId1"/></p:sldMasterIdLst>
  <p:sldIdLst>{slide_id_list}</p:sldIdLst>
  <p:sldSz cx="{slide_w}" cy="{slide_h}" type="wide"/>
  <p:notesSz cx="6858000" cy="9144000"/>
</p:presentation>'''
    presentation_rels_xml = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  {''.join(presentation_rels)}
</Relationships>'''
    slide_master = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldMaster xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
             xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
             xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:cSld><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
  </p:spTree></p:cSld>
  <p:sldLayoutIdLst><p:sldLayoutId id="2147483649" r:id="rId1"/></p:sldLayoutIdLst>
  <p:txStyles><p:titleStyle/><p:bodyStyle/><p:otherStyle/></p:txStyles>
</p:sldMaster>'''
    slide_master_rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/theme" Target="../theme/theme1.xml"/>
</Relationships>'''
    slide_layout = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldLayout xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
             xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
             xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main" type="blank" preserve="1">
  <p:cSld name="Blank"><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
  </p:spTree></p:cSld>
</p:sldLayout>'''
    slide_layout_rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="../slideMasters/slideMaster1.xml"/>
</Relationships>'''
    theme = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<a:theme xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" name="Simple">
  <a:themeElements>
    <a:clrScheme name="Simple">
      <a:dk1><a:srgbClr val="111827"/></a:dk1><a:lt1><a:srgbClr val="FFFFFF"/></a:lt1>
      <a:dk2><a:srgbClr val="334155"/></a:dk2><a:lt2><a:srgbClr val="F8FAFC"/></a:lt2>
      <a:accent1><a:srgbClr val="0F766E"/></a:accent1><a:accent2><a:srgbClr val="CA8A04"/></a:accent2>
      <a:accent3><a:srgbClr val="B45309"/></a:accent3><a:accent4><a:srgbClr val="475569"/></a:accent4>
      <a:accent5><a:srgbClr val="0369A1"/></a:accent5><a:accent6><a:srgbClr val="4D7C0F"/></a:accent6>
      <a:hlink><a:srgbClr val="2563EB"/></a:hlink><a:folHlink><a:srgbClr val="7C3AED"/></a:folHlink>
    </a:clrScheme>
    <a:fontScheme name="Simple"><a:majorFont><a:latin typeface="Aptos Display"/></a:majorFont><a:minorFont><a:latin typeface="Aptos"/></a:minorFont></a:fontScheme>
    <a:fmtScheme name="Simple"><a:fillStyleLst/><a:lnStyleLst/><a:effectStyleLst/><a:bgFillStyleLst/></a:fmtScheme>
  </a:themeElements>
</a:theme>'''

    with zipfile.ZipFile(out_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", content_types)
        zf.writestr("_rels/.rels", root_rels)
        zf.writestr("ppt/presentation.xml", presentation)
        zf.writestr("ppt/_rels/presentation.xml.rels", presentation_rels_xml)
        zf.writestr("ppt/slideMasters/slideMaster1.xml", slide_master)
        zf.writestr("ppt/slideMasters/_rels/slideMaster1.xml.rels", slide_master_rels)
        zf.writestr("ppt/slideLayouts/slideLayout1.xml", slide_layout)
        zf.writestr("ppt/slideLayouts/_rels/slideLayout1.xml.rels", slide_layout_rels)
        zf.writestr("ppt/theme/theme1.xml", theme)
        for i, slide in enumerate(slides, start=1):
            zf.writestr(f"ppt/slides/slide{i}.xml", slide)
            rels = slide_rels[i]
            zf.writestr(
                f"ppt/slides/_rels/slide{i}.xml.rels",
                f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">{rels}</Relationships>''',
            )
        for key, src in images.items():
            if not src.exists():
                continue
            zf.write(src, f"ppt/media/{media_names[key]}")


def create_report_pptx(out_path: Path, output_dir: Path, context: dict[str, object]):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    image_paths = {
        "raw": output_dir / "raw_point_cloud.png",
        "ptin": output_dir / "ground_ptin.png",
        "csf": output_dir / "ground_csf.png",
        "dem": output_dir / "dem_csf.png",
        "hist": output_dir / "height_histogram.png",
        "density": output_dir / "point_density_heatmap.png",
        "denoise": output_dir / "denoise_effect.png",
        "bar": output_dir / "filter_comparison_bar.png",
        "overlap": output_dir / "ptin_vs_csf_overlap.png",
        "hillshade": output_dir / "dem_hillshade.png",
        "contour": output_dir / "dem_contour.png",
        "difference": output_dir / "dem_difference.png",
        "profile": output_dir / "terrain_profile.png",
        "view3d": output_dir / "dem_3d_view.png",
    }
    media_names = {key: f"image{i}.png" for i, key in enumerate(image_paths, start=1)}

    def image_shape(shape_id: int, key: str, x: int, y: int, w: int, h: int, name: str | None = None):
        return pptx_xml_picture(shape_id, name or key, f"rId_{key}", x, y, w, h)

    def slide_title(text: str):
        return pptx_xml_textbox(
            2, "Title", 500_000, 250_000, 12_200_000, 450_000, text, 2600, True
        )

    ptin_ratio = 100.0 * context["ptin_count"] / max(context["point_count"], 1)
    csf_ratio = 100.0 * context["csf_count"] / max(context["point_count"], 1)
    overlap_ratio = 100.0 * context["both_ground_count"] / max(context["point_count"], 1)

    slides: list[tuple[list[str], list[str]]] = []
    slides.append(
        (
            [
                pptx_xml_textbox(
                    2,
                    "Title",
                    560_000,
                    360_000,
                    12_200_000,
                    650_000,
                    "激光点云地面滤波与 DEM 构建",
                    3400,
                    True,
                ),
                pptx_xml_textbox(
                    3,
                    "Subtitle",
                    700_000,
                    1_250_000,
                    11_500_000,
                    3_900_000,
                    (
                        f"数据文件：{context['las_name']}\n"
                        f"原始点数：{context['point_count']:,}\n"
                        f"方法：渐进式 TIN 滤波、布料滤波\n"
                        f"成果：地面点 LAS、GeoTIFF DEM、可视化分析图"
                    ),
                    2200,
                ),
            ],
            [],
        )
    )
    slides.append(
        (
            [
                pptx_xml_textbox(
                    2,
                    "Flow",
                    600_000,
                    540_000,
                    12_000_000,
                    6_000_000,
                    (
                        "技术流程\n"
                        "1. 读取 LAS 头信息和点记录，得到坐标、回波与分类字段。\n"
                        "2. 使用全局稳健高程阈值剔除明显噪声点。\n"
                        "3. 渐进式 TIN：最低点种子面 -> 逐级加密 -> 高差阈值判别地面点。\n"
                        "4. 布料滤波：高程取反 -> 布料下降碰撞 -> 反算地表面。\n"
                        "5. 对地面点栅格平均、空洞填补并输出 DEM GeoTIFF。\n"
                        "6. 输出统计图、差值图、剖面图和三维透视图用于报告分析。"
                    ),
                    2050,
                )
            ],
            [],
        )
    )
    slides.append(
        (
            [
                slide_title("原始点云俯视效果"),
                image_shape(3, "raw", 850_000, 850_000, 11_650_000, 5_950_000, "Raw"),
            ],
            ["raw"],
        )
    )
    slides.append(
        (
            [
                slide_title("点云统计分析"),
                image_shape(3, "hist", 550_000, 850_000, 5_950_000, 4_550_000, "Histogram"),
                image_shape(4, "density", 6_850_000, 850_000, 5_950_000, 4_550_000, "Density"),
                pptx_xml_textbox(
                    5,
                    "Note",
                    700_000,
                    5_650_000,
                    11_800_000,
                    760_000,
                    "高程直方图用于分析地面点主要高程区间；密度热力图用于判断扫描覆盖是否均匀。",
                    1650,
                ),
            ],
            ["hist", "density"],
        )
    )
    slides.append(
        (
            [
                slide_title("去噪与滤波数量分析"),
                image_shape(3, "denoise", 650_000, 900_000, 5_850_000, 4_650_000, "Denoise"),
                image_shape(4, "bar", 6_900_000, 900_000, 5_850_000, 4_650_000, "Bar"),
                pptx_xml_textbox(
                    5,
                    "Summary",
                    780_000,
                    5_760_000,
                    11_700_000,
                    680_000,
                    f"PTIN 地面点比例 {ptin_ratio:.2f}%，CSF 地面点比例 {csf_ratio:.2f}%。CSF 提取点数略多，PTIN 判别相对保守。",
                    1600,
                ),
            ],
            ["denoise", "bar"],
        )
    )
    slides.append(
        (
            [
                slide_title("两种地面滤波效果对比"),
                image_shape(3, "ptin", 650_000, 950_000, 5_950_000, 4_800_000, "PTIN"),
                image_shape(4, "csf", 6_750_000, 950_000, 5_950_000, 4_800_000, "CSF"),
                pptx_xml_textbox(5, "Label1", 1_050_000, 5_880_000, 5_200_000, 420_000, "渐进式 TIN 地面点", 1700, True),
                pptx_xml_textbox(6, "Label2", 7_250_000, 5_880_000, 5_200_000, 420_000, "布料滤波地面点", 1700, True),
            ],
            ["ptin", "csf"],
        )
    )
    slides.append(
        (
            [
                slide_title("PTIN 与 CSF 地面点差异"),
                image_shape(3, "overlap", 700_000, 850_000, 8_200_000, 5_700_000, "Overlap"),
                pptx_xml_textbox(
                    4,
                    "OverlapText",
                    9_250_000,
                    1_150_000,
                    3_400_000,
                    4_800_000,
                    (
                        f"共同地面点：{context['both_ground_count']:,}\n"
                        f"共同占比：{overlap_ratio:.2f}%\n"
                        f"PTIN only：{context['ptin_only_count']:,}\n"
                        f"CSF only：{context['csf_only_count']:,}\n"
                        "共同区域代表稳定地表，单方法区域体现算法差异。"
                    ),
                    1700,
                ),
            ],
            ["overlap"],
        )
    )
    slides.append(
        (
            [
                slide_title("DEM 成果"),
                image_shape(3, "dem", 900_000, 900_000, 7_800_000, 5_700_000, "DEM"),
                pptx_xml_textbox(
                    4,
                    "Summary",
                    9_050_000,
                    1_150_000,
                    3_700_000,
                    4_800_000,
                    (
                        f"DEM 分辨率：{DEM_RESOLUTION} m\n"
                        f"PTIN 地面点：{context['ptin_count']:,}\n"
                        f"CSF 地面点：{context['csf_count']:,}\n"
                        "GeoTIFF 使用 LAS 本地坐标写入像元比例尺和定位标签。"
                    ),
                    1850,
                ),
            ],
            ["dem"],
        )
    )
    slides.append(
        (
            [
                slide_title("DEM 阴影与等高线分析"),
                image_shape(3, "hillshade", 650_000, 900_000, 5_850_000, 4_750_000, "Hillshade"),
                image_shape(4, "contour", 6_900_000, 900_000, 5_850_000, 4_750_000, "Contour"),
                pptx_xml_textbox(
                    5,
                    "Note",
                    780_000,
                    5_850_000,
                    11_700_000,
                    600_000,
                    "阴影图突出地形起伏和坡面形态，等高线图用于检查高程连续性和局部坡度变化。",
                    1600,
                ),
            ],
            ["hillshade", "contour"],
        )
    )
    slides.append(
        (
            [
                slide_title("两种 DEM 差值分析"),
                image_shape(3, "difference", 800_000, 900_000, 7_700_000, 5_500_000, "Difference"),
                pptx_xml_textbox(
                    4,
                    "DiffText",
                    8_900_000,
                    1_200_000,
                    3_900_000,
                    4_600_000,
                    (
                        "差值定义：CSF DEM - PTIN DEM\n"
                        f"均值：{context['dem_diff_mean']:.3f} m\n"
                        f"标准差：{context['dem_diff_std']:.3f} m\n"
                        f"95% 绝对差值：{context['dem_diff_abs_p95']:.3f} m\n"
                        "红色表示 CSF 高于 PTIN，蓝色表示 CSF 低于 PTIN。"
                    ),
                    1750,
                ),
            ],
            ["difference"],
        )
    )
    slides.append(
        (
            [
                slide_title("剖面与三维地形展示"),
                image_shape(3, "profile", 650_000, 900_000, 5_850_000, 4_750_000, "Profile"),
                image_shape(4, "view3d", 6_900_000, 900_000, 5_850_000, 4_750_000, "3D"),
                pptx_xml_textbox(
                    5,
                    "Note",
                    780_000,
                    5_850_000,
                    11_700_000,
                    600_000,
                    "剖面图用于比较两种 DEM 的局部一致性，伪三维图用于展示地形级三维成果形态。",
                    1600,
                ),
            ],
            ["profile", "view3d"],
        )
    )

    slide_xmls = [make_slide_xml(shapes) for shapes, _ in slides]
    slide_rels: dict[int, str] = {}
    used_media = set()
    for i, (_, keys) in enumerate(slides, start=1):
        rels = []
        for key in keys:
            rels.append(
                f'<Relationship Id="rId_{key}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="../media/{media_names[key]}"/>'
            )
            used_media.add(key)
        slide_rels[i] = "".join(rels)

    slide_w = 13_333_500
    slide_h = 7_500_000
    presentation_rels = [
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="slideMasters/slideMaster1.xml"/>'
    ]
    for i in range(1, len(slide_xmls) + 1):
        presentation_rels.append(
            f'<Relationship Id="rId{i + 1}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide" Target="slides/slide{i}.xml"/>'
        )
    slide_id_list = "\n".join(
        f'<p:sldId id="{255 + i}" r:id="rId{i + 1}"/>' for i in range(1, len(slide_xmls) + 1)
    )
    overrides = "\n".join(
        f'<Override PartName="/ppt/slides/slide{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slide+xml"/>'
        for i in range(1, len(slide_xmls) + 1)
    )
    content_types = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Default Extension="png" ContentType="image/png"/>
  <Override PartName="/ppt/presentation.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.presentation.main+xml"/>
  <Override PartName="/ppt/slideMasters/slideMaster1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideMaster+xml"/>
  <Override PartName="/ppt/slideLayouts/slideLayout1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideLayout+xml"/>
  <Override PartName="/ppt/theme/theme1.xml" ContentType="application/vnd.openxmlformats-officedocument.theme+xml"/>
  {overrides}
</Types>'''
    root_rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="ppt/presentation.xml"/>
</Relationships>'''
    presentation = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:presentation xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
                xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
                xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:sldMasterIdLst><p:sldMasterId id="2147483648" r:id="rId1"/></p:sldMasterIdLst>
  <p:sldIdLst>{slide_id_list}</p:sldIdLst>
  <p:sldSz cx="{slide_w}" cy="{slide_h}" type="wide"/>
  <p:notesSz cx="6858000" cy="9144000"/>
</p:presentation>'''
    presentation_rels_xml = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  {''.join(presentation_rels)}
</Relationships>'''
    slide_master = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldMaster xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
             xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
             xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:cSld><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
  </p:spTree></p:cSld>
  <p:sldLayoutIdLst><p:sldLayoutId id="2147483649" r:id="rId1"/></p:sldLayoutIdLst>
  <p:txStyles><p:titleStyle/><p:bodyStyle/><p:otherStyle/></p:txStyles>
</p:sldMaster>'''
    slide_master_rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/theme" Target="../theme/theme1.xml"/>
</Relationships>'''
    slide_layout = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldLayout xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
             xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
             xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main" type="blank" preserve="1">
  <p:cSld name="Blank"><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
  </p:spTree></p:cSld>
</p:sldLayout>'''
    slide_layout_rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="../slideMasters/slideMaster1.xml"/>
</Relationships>'''
    theme = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<a:theme xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" name="Simple">
  <a:themeElements>
    <a:clrScheme name="Simple">
      <a:dk1><a:srgbClr val="111827"/></a:dk1><a:lt1><a:srgbClr val="FFFFFF"/></a:lt1>
      <a:dk2><a:srgbClr val="334155"/></a:dk2><a:lt2><a:srgbClr val="F8FAFC"/></a:lt2>
      <a:accent1><a:srgbClr val="0F766E"/></a:accent1><a:accent2><a:srgbClr val="CA8A04"/></a:accent2>
      <a:accent3><a:srgbClr val="B45309"/></a:accent3><a:accent4><a:srgbClr val="475569"/></a:accent4>
      <a:accent5><a:srgbClr val="0369A1"/></a:accent5><a:accent6><a:srgbClr val="4D7C0F"/></a:accent6>
      <a:hlink><a:srgbClr val="2563EB"/></a:hlink><a:folHlink><a:srgbClr val="7C3AED"/></a:folHlink>
    </a:clrScheme>
    <a:fontScheme name="Simple"><a:majorFont><a:latin typeface="Aptos Display"/></a:majorFont><a:minorFont><a:latin typeface="Aptos"/></a:minorFont></a:fontScheme>
    <a:fmtScheme name="Simple"><a:fillStyleLst/><a:lnStyleLst/><a:effectStyleLst/><a:bgFillStyleLst/></a:fmtScheme>
  </a:themeElements>
</a:theme>'''

    with zipfile.ZipFile(out_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", content_types)
        zf.writestr("_rels/.rels", root_rels)
        zf.writestr("ppt/presentation.xml", presentation)
        zf.writestr("ppt/_rels/presentation.xml.rels", presentation_rels_xml)
        zf.writestr("ppt/slideMasters/slideMaster1.xml", slide_master)
        zf.writestr("ppt/slideMasters/_rels/slideMaster1.xml.rels", slide_master_rels)
        zf.writestr("ppt/slideLayouts/slideLayout1.xml", slide_layout)
        zf.writestr("ppt/slideLayouts/_rels/slideLayout1.xml.rels", slide_layout_rels)
        zf.writestr("ppt/theme/theme1.xml", theme)
        for i, slide in enumerate(slide_xmls, start=1):
            zf.writestr(f"ppt/slides/slide{i}.xml", slide)
            zf.writestr(
                f"ppt/slides/_rels/slide{i}.xml.rels",
                f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">{slide_rels[i]}</Relationships>''',
            )
        for key in sorted(used_media):
            src = image_paths[key]
            if src.exists():
                zf.write(src, f"ppt/media/{media_names[key]}")


# ---------------------------------------------------------------------------
# Project wiring
# ---------------------------------------------------------------------------


def find_las_file() -> Path:
    here = Path(__file__).resolve().parent
    candidates = [
        here / "data" / SELECTED_LAS_NAME,
        here.parent / "data" / SELECTED_LAS_NAME,
        here.parent.parent / "数据" / "Hwdata" / SELECTED_LAS_NAME,
        here.parent.parent / "方象" / "Hwdata" / SELECTED_LAS_NAME,
        Path.home()
        / "OneDrive"
        / "Desktop"
        / "三维建模"
        / "数据"
        / "Hwdata"
        / SELECTED_LAS_NAME,
        Path.home()
        / "Desktop"
        / "三维建模"
        / "数据"
        / "Hwdata"
        / SELECTED_LAS_NAME,
    ]
    for path in candidates:
        if path.exists():
            return path

    search_roots = [here, here.parent, here.parent.parent, Path.cwd()]
    seen = set()
    for root in search_roots:
        try:
            root = root.resolve()
        except OSError:
            continue
        if root in seen or not root.exists():
            continue
        seen.add(root)
        matches = list(root.rglob(SELECTED_LAS_NAME))
        if matches:
            return matches[0]

    raise FileNotFoundError(
        f"Cannot find {SELECTED_LAS_NAME}. Put it in a data folder beside main.py "
        "or keep the original Desktop/三维建模/数据/Hwdata directory."
    )


def write_readme(project_dir: Path):
    readme = """# 激光点云地面滤波与 DEM 构建作业

## 运行方法

1. 用 PyCharm 打开本文件夹。
2. 选择 Python 3 解释器，并确保已安装 `numpy`。
3. 直接运行 `main.py`。

程序会自动寻找 `20251126150027848.las`。如果找不到，请把该 LAS 文件放到：

```text
data/20251126150027848.las
```

运行完成后，所有成果会生成在 `outputs/` 目录。

## 主要成果

- `ground_ptin.las`：渐进式 TIN 滤波地面点。
- `ground_csf.las`：布料滤波地面点。
- `dem_ptin.tif`、`dem_csf.tif`：DEM GeoTIFF。
- `raw_point_cloud.png`、`ground_ptin.png`、`ground_csf.png`、`dem_csf.png`：报告截图。
- `homework_report.pptx`：简明实验报告 PPT。

说明：源 LAS 文件没有投影 VLR 信息，因此 GeoTIFF 写入的是 LAS 本地坐标系下的像元比例尺与定位标签。
"""
    (project_dir / "README.md").write_text(readme, encoding="utf-8")
    (project_dir / "requirements.txt").write_text("numpy>=1.22\n", encoding="ascii")


def main():
    started = time.time()
    project_dir = Path(__file__).resolve().parent
    output_dir = project_dir / OUTPUT_DIR_NAME
    output_dir.mkdir(parents=True, exist_ok=True)
    write_readme(project_dir)

    las_path = find_las_file()
    header = read_las_header(las_path)
    if header.point_format not in {0, 1, 2, 3, 4, 5}:
        raise ValueError(
            f"Point format {header.point_format} is not supported by this lightweight script."
        )

    print("=" * 72)
    print("Laser point-cloud ground filtering homework")
    print(f"Input: {las_path}")
    print(
        f"LAS {header.version_major}.{header.version_minor}, "
        f"format {header.point_format}, points {header.point_count:,}"
    )
    print(
        f"Bounds: X[{header.xmin:.3f}, {header.xmax:.3f}], "
        f"Y[{header.ymin:.3f}, {header.ymax:.3f}], "
        f"Z[{header.zmin:.3f}, {header.zmax:.3f}]"
    )
    print("=" * 72)

    x, y, z, return_flags = load_points(header)

    z_low, z_high = np.percentile(z, [0.05, 99.95])
    denoise_mask = (z >= z_low - 0.35) & (z <= z_high + 0.35)
    print(
        f"Denoising height gate: [{z_low - 0.35:.3f}, {z_high + 0.35:.3f}], "
        f"kept {int(denoise_mask.sum()):,}/{len(z):,}"
    )

    ptin_surface, ptin_spec = build_progressive_tin_surface(x, y, z, denoise_mask)
    ptin_mask = classify_ground(
        "PTIN",
        ptin_surface,
        ptin_spec,
        x,
        y,
        z,
        denoise_mask,
        PTIN_FINAL_ABOVE,
        PTIN_FINAL_BELOW,
    )

    csf_surface, csf_spec = build_cloth_surface(x, y, z, denoise_mask)
    csf_mask = classify_ground(
        "CSF",
        csf_surface,
        csf_spec,
        x,
        y,
        z,
        denoise_mask,
        CSF_FINAL_ABOVE,
        CSF_FINAL_BELOW,
    )

    write_ground_las(
        header, output_dir / "ground_ptin.las", x, y, z, return_flags, ptin_mask
    )
    write_ground_las(
        header, output_dir / "ground_csf.las", x, y, z, return_flags, csf_mask
    )

    dem_ptin, dem_spec = build_dem(x, y, z, ptin_mask)
    dem_csf, _ = build_dem(x, y, z, csf_mask)
    write_geotiff_float32(output_dir / "dem_ptin.tif", dem_ptin, dem_spec)
    write_geotiff_float32(output_dir / "dem_csf.tif", dem_csf, dem_spec)
    print(f"Wrote DEM GeoTIFF files at {DEM_RESOLUTION}m resolution")

    raw_idx = sample_indices(len(x))
    bounds_2d = (float(x.min()), float(x.max()), float(y.min()), float(y.max()))
    render_point_map(
        output_dir / "raw_point_cloud.png",
        x[raw_idx],
        y[raw_idx],
        z[raw_idx],
        bounds_2d,
    )

    ptin_idx = sample_indices(int(ptin_mask.sum()))
    csf_idx = sample_indices(int(csf_mask.sum()))
    ptin_points = np.flatnonzero(ptin_mask)[ptin_idx]
    csf_points = np.flatnonzero(csf_mask)[csf_idx]
    render_point_map(
        output_dir / "ground_ptin.png",
        x[ptin_points],
        y[ptin_points],
        z[ptin_points],
        bounds_2d,
        background=(x[raw_idx], y[raw_idx]),
    )
    render_point_map(
        output_dir / "ground_csf.png",
        x[csf_points],
        y[csf_points],
        z[csf_points],
        bounds_2d,
        background=(x[raw_idx], y[raw_idx]),
    )
    render_dem(output_dir / "dem_ptin.png", dem_ptin)
    render_dem(output_dir / "dem_csf.png", dem_csf)

    analysis_stats = render_analysis_figures(
        output_dir,
        x,
        y,
        z,
        denoise_mask,
        ptin_mask,
        csf_mask,
        dem_ptin,
        dem_csf,
        dem_spec,
        bounds_2d,
    )

    context = {
        "las_name": header.path.name,
        "point_count": len(x),
        "denoised_count": int(denoise_mask.sum()),
        "ptin_count": int(ptin_mask.sum()),
        "csf_count": int(csf_mask.sum()),
    }
    context.update(analysis_stats)
    write_flowchart_svg(output_dir / "algorithm_flowchart.svg")
    write_report_notes(output_dir / "report_notes.md", context)
    create_report_pptx(output_dir / "homework_report.pptx", output_dir, context)

    print("=" * 72)
    print(f"All results saved to: {output_dir}")
    print(f"Total time: {time.time() - started:.1f}s")
    print("Done.")


if __name__ == "__main__":
    try:
        main()
    except ImportError as exc:
        print("Missing dependency:", exc)
        print("Install it with: pip install -r requirements.txt")
        raise
