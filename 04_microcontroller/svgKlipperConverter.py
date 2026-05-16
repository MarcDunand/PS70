#!/usr/bin/env python3
"""
svgKlipperConverter.py

SVG-to-Klipper G-code converter for an XY marbling / plotting machine.

This version supports two kinds of SVG actions:
    1. Draw actions: normal SVG paths/shapes are traced with the stick, like a pen plotter.
    2. Pump actions: small colored circle markers are interpreted as ink drops.

Layer behavior:
    - Layers/groups are only used for chronological ordering.
    - You can mix drawing geometry and pump markers on the same layer.
    - If a layer/group name starts with a number, layers are processed by that number.
      Example: 00_first, 10_second, 20_third.
    - If no numeric prefix is present, normal SVG document order is used.

Pump marker behavior:
    - A marker is any circle whose fill/stroke color matches one of the pump marker colors.
    - The marker color determines which pump fires.
    - Marker circles are not drawn as geometry.
    - Circle radius/diameter is ignored for pump detection.
    - Circles with unrecognized colors are treated as normal drawable geometry.

Expected workflow:
    1. In Illustrator/Inkscape/etc., convert artwork to paths when possible.
    2. Add 1 mm colored circles wherever you want ink drops.
    3. Save as a plain SVG.
    4. Run this script to create a .gcode file.
    5. Upload/run the .gcode file through Mainsail/Fluidd/Klipper.

Example:
    python svgKlipperConverter.py RadialDesign.svg

Example with explicit output:
    python svgKlipperConverter.py RadialDesign.svg -o radial_test.gcode

Notes:
    - No third-party Python packages are required.
    - G-code feed rates are in mm/min.
    - This script outputs normal X/Y coordinates. Klipper handles CoreXY motor mixing.
    - Supported SVG primitives: path, line, polyline, polygon, circle, ellipse, rect.
    - Supported path commands: M, L, H, V, C, S, Q, T, A, Z. Curves/arcs are flattened.
    - Basic SVG transforms are supported: matrix, translate, scale, rotate.
"""

from __future__ import annotations

import argparse
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]
Polyline = List[Point]
Matrix = Tuple[float, float, float, float, float, float]  # SVG affine: a b c d e f

# -----------------------------------------------------------------------------
# Pump marker configuration
# -----------------------------------------------------------------------------

# Exact marker colors. The converter checks fill first, then stroke if fill is absent/none.
# Keep these as simple hex colors for the most reliable Illustrator/Inkscape workflow.
PUMP_MARKER_COLORS: Dict[str, int] = {
    "#0000ff": 0,  # blue
    "#ff0000": 1,  # red
    "#00aa00": 2,  # green
    "#000000": 3,  # black
}

# A pump marker is a circle whose SVG/design-space diameter is about this size.
# Marker circles are symbols and are not physically drawn, so their detection is
# intentionally independent of the final plot scaling.
PUMP_MARKER_DIAMETER_MM = 1.0
PUMP_MARKER_DIAMETER_TOLERANCE_MM = 0.15

# Pump timing. DOSE already includes the active pump pulse duration internally;
# this converter then adds a separate settling / movement delay after the pulse.
PUMP_PULSE_SECONDS = 0.100
PUMP_SETTLE_DELAY_MS = 250

# Stick timing. These delays are emitted manually after STICK_UP and STICK_DOWN.
STICK_SETTLE_DELAY_MS = 150


@dataclass
class ToolOffset:
    x_mm: float = 0.0
    y_mm: float = 0.0


@dataclass
class ConverterConfig:
    # Effective machine working area.
    machine_width_mm: float = 270.0
    machine_height_mm: float = 430.0

    # Target maximum design size. By default, use the full effective working area.
    # The fitter also accounts for stick/pump offsets so future nonzero offsets
    # do not push the actual active tool position outside the machine area.
    plot_width_mm: float = 270.0
    plot_height_mm: float = 430.0

    # Extra margin from machine boundaries. Default is 0 because the effective
    # working area already accounts for usable travel.
    margin_mm: float = 0.0

    # Global offset applied after centering/fitting.
    offset_x_mm: float = 0.0
    offset_y_mm: float = 0.0

    # Tool offsets are measured from the machine's XY reference point to the
    # actual active point of the stick/pump. They are all zero for now, but are
    # included in fitting and output so calibration can be added later.
    stick_offset: ToolOffset = field(default_factory=lambda: ToolOffset(35, 0))
    pump_offsets: Dict[int, ToolOffset] = field(default_factory=lambda: {
        0: ToolOffset(-70.0, -50.0),
        1: ToolOffset(-70.0, -20.0),
        2: ToolOffset(-0.0, -20.0),
        3: ToolOffset(-0.0, -50.0),
    })

    # Curve sampling: smaller = smoother curves/circles, more G-code lines.
    sample_step_svg_units: float = 4.0

    # If True, SVG Y-down coordinates become machine Y-up coordinates.
    flip_y: bool = True

    # Motion settings.
    draw_feed_mm_min: float = 5000.0       # 83.3 mm/s
    travel_feed_mm_min: float = 50000.0    # 166.7 mm/s

    # Optional Klipper velocity limits. Set either to None to avoid emitting it.
    max_velocity_mm_s: Optional[float] = None
    max_accel_mm_s2: Optional[float] = None

    # Safety / convenience options.
    include_home: bool = False
    return_to_origin: bool = True
    include_m84: bool = False

    # Remove nearly duplicate points to reduce tiny planner moves.
    min_segment_mm: float = 0.05

    # Pump marker parameters copied from the top-level constants so they can be
    # changed by CLI if needed.
    pump_marker_diameter_mm: float = PUMP_MARKER_DIAMETER_MM
    pump_marker_tolerance_mm: float = PUMP_MARKER_DIAMETER_TOLERANCE_MM
    pump_pulse_seconds: float = PUMP_PULSE_SECONDS
    pump_settle_delay_ms: int = PUMP_SETTLE_DELAY_MS
    stick_settle_delay_ms: int = STICK_SETTLE_DELAY_MS


@dataclass
class SvgItem:
    kind: str                       # "path", "line", "polyline", "polygon", "circle", "ellipse", "rect"
    polylines: List[Polyline]
    center: Optional[Point] = None  # circle/ellipse/rect center, useful for pump markers
    diameter_svg_units: Optional[float] = None
    color: Optional[str] = None
    source_tag: str = ""
    layer_name: str = ""
    layer_sort: Tuple[float, int] = (0.0, 0)
    element_order: int = 0


@dataclass
class Action:
    kind: str                      # "draw" or "pump"
    layer_sort: Tuple[float, int]
    element_order: int
    polylines: List[Polyline] = field(default_factory=list)
    point: Optional[Point] = None
    pump: Optional[int] = None
    source_tag: str = ""
    layer_name: str = ""


# -------------------- basic SVG parsing helpers --------------------

_NUM_RE = re.compile(r"[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?")
_PATH_TOKEN_RE = re.compile(r"[MmLlHhVvCcSsQqTtAaZz]|[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?")
_TRANSFORM_RE = re.compile(r"(matrix|translate|scale|rotate)\s*\(([^)]*)\)")
_STYLE_SPLIT_RE = re.compile(r"\s*;\s*")
_LAYER_NUM_RE = re.compile(r"^\s*([-+]?\d+(?:\.\d+)?)")


def strip_ns(tag: str) -> str:
    return tag.split("}", 1)[-1] if "}" in tag else tag


def parse_float(value: Optional[str], default: float = 0.0) -> float:
    if value is None:
        return default
    match = _NUM_RE.search(value)
    if not match:
        return default
    return float(match.group(0))


def parse_points(value: str) -> Polyline:
    nums = [float(x) for x in _NUM_RE.findall(value or "")]
    pts: Polyline = []
    for i in range(0, len(nums) - 1, 2):
        pts.append((nums[i], nums[i + 1]))
    return pts


def distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def add_point(poly: Polyline, pt: Point, min_dist: float = 0.0) -> None:
    if poly and distance(poly[-1], pt) < min_dist:
        return
    poly.append(pt)


def cubic(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    u = 1.0 - t
    return (
        u**3 * p0[0] + 3 * u**2 * t * p1[0] + 3 * u * t**2 * p2[0] + t**3 * p3[0],
        u**3 * p0[1] + 3 * u**2 * t * p1[1] + 3 * u * t**2 * p2[1] + t**3 * p3[1],
    )


def quadratic(p0: Point, p1: Point, p2: Point, t: float) -> Point:
    u = 1.0 - t
    return (
        u**2 * p0[0] + 2 * u * t * p1[0] + t**2 * p2[0],
        u**2 * p0[1] + 2 * u * t * p1[1] + t**2 * p2[1],
    )


def flatten_cubic(p0: Point, p1: Point, p2: Point, p3: Point, step: float) -> Polyline:
    approx_len = distance(p0, p1) + distance(p1, p2) + distance(p2, p3)
    n = max(4, int(math.ceil(approx_len / max(step, 0.1))))
    return [cubic(p0, p1, p2, p3, i / n) for i in range(1, n + 1)]


def flatten_quadratic(p0: Point, p1: Point, p2: Point, step: float) -> Polyline:
    approx_len = distance(p0, p1) + distance(p1, p2)
    n = max(4, int(math.ceil(approx_len / max(step, 0.1))))
    return [quadratic(p0, p1, p2, i / n) for i in range(1, n + 1)]


def vector_angle(u: Point, v: Point) -> float:
    dot = u[0] * v[0] + u[1] * v[1]
    det = u[0] * v[1] - u[1] * v[0]
    return math.atan2(det, dot)


def flatten_arc(
    p0: Point,
    rx: float,
    ry: float,
    x_axis_rotation_deg: float,
    large_arc_flag: int,
    sweep_flag: int,
    p1: Point,
    step: float,
) -> Polyline:
    """Flatten an SVG elliptical arc command into points. Based on SVG arc implementation notes."""
    if rx == 0 or ry == 0 or p0 == p1:
        return [p1]

    rx = abs(rx)
    ry = abs(ry)
    phi = math.radians(x_axis_rotation_deg % 360.0)
    cos_phi = math.cos(phi)
    sin_phi = math.sin(phi)

    dx = (p0[0] - p1[0]) / 2.0
    dy = (p0[1] - p1[1]) / 2.0

    x1p = cos_phi * dx + sin_phi * dy
    y1p = -sin_phi * dx + cos_phi * dy

    lam = (x1p**2) / (rx**2) + (y1p**2) / (ry**2)
    if lam > 1:
        scale = math.sqrt(lam)
        rx *= scale
        ry *= scale

    sign = -1.0 if large_arc_flag == sweep_flag else 1.0
    numerator = rx**2 * ry**2 - rx**2 * y1p**2 - ry**2 * x1p**2
    denominator = rx**2 * y1p**2 + ry**2 * x1p**2
    coef = sign * math.sqrt(max(0.0, numerator / denominator)) if denominator else 0.0

    cxp = coef * (rx * y1p / ry)
    cyp = coef * (-ry * x1p / rx)

    cx = cos_phi * cxp - sin_phi * cyp + (p0[0] + p1[0]) / 2.0
    cy = sin_phi * cxp + cos_phi * cyp + (p0[1] + p1[1]) / 2.0

    theta1 = vector_angle((1.0, 0.0), ((x1p - cxp) / rx, (y1p - cyp) / ry))
    delta = vector_angle(
        ((x1p - cxp) / rx, (y1p - cyp) / ry),
        ((-x1p - cxp) / rx, (-y1p - cyp) / ry),
    )

    if not sweep_flag and delta > 0:
        delta -= 2.0 * math.pi
    elif sweep_flag and delta < 0:
        delta += 2.0 * math.pi

    approx_len = max(rx, ry) * abs(delta)
    n = max(4, int(math.ceil(approx_len / max(step, 0.1))))

    pts: Polyline = []
    for i in range(1, n + 1):
        theta = theta1 + delta * (i / n)
        x = cx + rx * math.cos(theta) * cos_phi - ry * math.sin(theta) * sin_phi
        y = cy + rx * math.cos(theta) * sin_phi + ry * math.sin(theta) * cos_phi
        pts.append((x, y))
    return pts


def tokenize_path(d: str) -> List[str]:
    return _PATH_TOKEN_RE.findall(d or "")


def is_command(token: str) -> bool:
    return len(token) == 1 and token.isalpha()


# -------------------- basic SVG transform helpers --------------------


def mat_identity() -> Matrix:
    return (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def mat_mul(m1: Matrix, m2: Matrix) -> Matrix:
    a1, b1, c1, d1, e1, f1 = m1
    a2, b2, c2, d2, e2, f2 = m2
    return (
        a1 * a2 + c1 * b2,
        b1 * a2 + d1 * b2,
        a1 * c2 + c1 * d2,
        b1 * c2 + d1 * d2,
        a1 * e2 + c1 * f2 + e1,
        b1 * e2 + d1 * f2 + f1,
    )


def apply_mat(m: Matrix, p: Point) -> Point:
    a, b, c, d, e, f = m
    return (a * p[0] + c * p[1] + e, b * p[0] + d * p[1] + f)


def parse_transform(transform: Optional[str]) -> Matrix:
    if not transform:
        return mat_identity()
    out = mat_identity()
    for name, args_text in _TRANSFORM_RE.findall(transform):
        nums = [float(x) for x in _NUM_RE.findall(args_text)]
        m = mat_identity()
        if name == "matrix" and len(nums) >= 6:
            m = (nums[0], nums[1], nums[2], nums[3], nums[4], nums[5])
        elif name == "translate" and nums:
            tx = nums[0]
            ty = nums[1] if len(nums) > 1 else 0.0
            m = (1.0, 0.0, 0.0, 1.0, tx, ty)
        elif name == "scale" and nums:
            sx = nums[0]
            sy = nums[1] if len(nums) > 1 else sx
            m = (sx, 0.0, 0.0, sy, 0.0, 0.0)
        elif name == "rotate" and nums:
            angle = math.radians(nums[0])
            ca = math.cos(angle)
            sa = math.sin(angle)
            rot = (ca, sa, -sa, ca, 0.0, 0.0)
            if len(nums) >= 3:
                cx, cy = nums[1], nums[2]
                m = mat_mul(mat_mul((1.0, 0.0, 0.0, 1.0, cx, cy), rot), (1.0, 0.0, 0.0, 1.0, -cx, -cy))
            else:
                m = rot
        out = mat_mul(out, m)
    return out


def transform_polyline(poly: Polyline, m: Matrix) -> Polyline:
    return [apply_mat(m, p) for p in poly]


# -------------------- style / color helpers --------------------


def parse_style_attr(style: Optional[str]) -> Dict[str, str]:
    result: Dict[str, str] = {}
    if not style:
        return result
    for part in _STYLE_SPLIT_RE.split(style.strip()):
        if not part or ":" not in part:
            continue
        key, value = part.split(":", 1)
        result[key.strip().lower()] = value.strip()
    return result


def merge_style(parent: Dict[str, str], elem: ET.Element) -> Dict[str, str]:
    style = dict(parent)
    style.update(parse_style_attr(elem.attrib.get("style")))
    for key in ("fill", "stroke"):
        if key in elem.attrib:
            style[key] = elem.attrib[key]
    return style


def normalize_color(value: Optional[str]) -> Optional[str]:
    if value is None:
        return None
    v = value.strip().lower()
    if not v or v == "none":
        return None
    if v.startswith("#"):
        hexpart = v[1:]
        if len(hexpart) == 3:
            hexpart = "".join(ch * 2 for ch in hexpart)
        if len(hexpart) == 6 and re.fullmatch(r"[0-9a-f]{6}", hexpart):
            return f"#{hexpart}"
    rgb_match = re.fullmatch(r"rgb\s*\(\s*([0-9.]+%?)\s*,\s*([0-9.]+%?)\s*,\s*([0-9.]+%?)\s*\)", v)
    if rgb_match:
        vals = []
        for group in rgb_match.groups():
            if group.endswith("%"):
                vals.append(max(0, min(255, round(float(group[:-1]) * 2.55))))
            else:
                vals.append(max(0, min(255, round(float(group)))))
        return f"#{vals[0]:02x}{vals[1]:02x}{vals[2]:02x}"
    named = {
        "black": "#000000",
        "blue": "#0000ff",
        "red": "#ff0000",
        "green": "#008000",
        "white": "#ffffff",
    }
    return named.get(v)


def marker_color_from_style(style: Dict[str, str]) -> Optional[str]:
    # Prefer fill. If the marker is stroke-only, use stroke.
    fill = normalize_color(style.get("fill"))
    if fill is not None:
        return fill
    return normalize_color(style.get("stroke"))


# -------------------- path parsing --------------------


def parse_path_d(d: str, step: float) -> List[Polyline]:
    """Parse an SVG path d string into one or more flattened polylines."""
    tokens = tokenize_path(d)
    if not tokens:
        return []

    i = 0
    cmd: Optional[str] = None
    current: Point = (0.0, 0.0)
    start: Point = (0.0, 0.0)
    last_cubic_ctrl: Optional[Point] = None
    last_quad_ctrl: Optional[Point] = None
    polylines: List[Polyline] = []
    poly: Polyline = []

    def has_num() -> bool:
        return i < len(tokens) and not is_command(tokens[i])

    def read_num() -> float:
        nonlocal i
        if i >= len(tokens) or is_command(tokens[i]):
            raise ValueError("Unexpected end of path command")
        v = float(tokens[i])
        i += 1
        return v

    def finish_poly() -> None:
        nonlocal poly
        if len(poly) >= 2:
            polylines.append(poly)
        poly = []

    while i < len(tokens):
        if is_command(tokens[i]):
            cmd = tokens[i]
            i += 1
        if cmd is None:
            raise ValueError("Path data starts with numbers instead of a command")

        absolute = cmd.isupper()
        c = cmd.upper()

        if c == "M":
            x = read_num(); y = read_num()
            current = (x, y) if absolute else (current[0] + x, current[1] + y)
            finish_poly()
            poly = [current]
            start = current
            last_cubic_ctrl = None
            last_quad_ctrl = None

            while has_num():
                x = read_num(); y = read_num()
                current = (x, y) if absolute else (current[0] + x, current[1] + y)
                add_point(poly, current)
            cmd = "L" if absolute else "l"

        elif c == "L":
            while has_num():
                x = read_num(); y = read_num()
                current = (x, y) if absolute else (current[0] + x, current[1] + y)
                add_point(poly, current)
            last_cubic_ctrl = None
            last_quad_ctrl = None

        elif c == "H":
            while has_num():
                x = read_num()
                current = (x, current[1]) if absolute else (current[0] + x, current[1])
                add_point(poly, current)
            last_cubic_ctrl = None
            last_quad_ctrl = None

        elif c == "V":
            while has_num():
                y = read_num()
                current = (current[0], y) if absolute else (current[0], current[1] + y)
                add_point(poly, current)
            last_cubic_ctrl = None
            last_quad_ctrl = None

        elif c == "C":
            while has_num():
                x1 = read_num(); y1 = read_num(); x2 = read_num(); y2 = read_num(); x = read_num(); y = read_num()
                p1 = (x1, y1) if absolute else (current[0] + x1, current[1] + y1)
                p2 = (x2, y2) if absolute else (current[0] + x2, current[1] + y2)
                p3 = (x, y) if absolute else (current[0] + x, current[1] + y)
                for pt in flatten_cubic(current, p1, p2, p3, step):
                    add_point(poly, pt)
                current = p3
                last_cubic_ctrl = p2
                last_quad_ctrl = None

        elif c == "S":
            while has_num():
                if last_cubic_ctrl is None:
                    p1 = current
                else:
                    p1 = (2 * current[0] - last_cubic_ctrl[0], 2 * current[1] - last_cubic_ctrl[1])
                x2 = read_num(); y2 = read_num(); x = read_num(); y = read_num()
                p2 = (x2, y2) if absolute else (current[0] + x2, current[1] + y2)
                p3 = (x, y) if absolute else (current[0] + x, current[1] + y)
                for pt in flatten_cubic(current, p1, p2, p3, step):
                    add_point(poly, pt)
                current = p3
                last_cubic_ctrl = p2
                last_quad_ctrl = None

        elif c == "Q":
            while has_num():
                x1 = read_num(); y1 = read_num(); x = read_num(); y = read_num()
                p1 = (x1, y1) if absolute else (current[0] + x1, current[1] + y1)
                p2 = (x, y) if absolute else (current[0] + x, current[1] + y)
                for pt in flatten_quadratic(current, p1, p2, step):
                    add_point(poly, pt)
                current = p2
                last_quad_ctrl = p1
                last_cubic_ctrl = None

        elif c == "T":
            while has_num():
                if last_quad_ctrl is None:
                    p1 = current
                else:
                    p1 = (2 * current[0] - last_quad_ctrl[0], 2 * current[1] - last_quad_ctrl[1])
                x = read_num(); y = read_num()
                p2 = (x, y) if absolute else (current[0] + x, current[1] + y)
                for pt in flatten_quadratic(current, p1, p2, step):
                    add_point(poly, pt)
                current = p2
                last_quad_ctrl = p1
                last_cubic_ctrl = None

        elif c == "A":
            while has_num():
                rx = read_num(); ry = read_num(); rot = read_num(); laf = int(read_num()); sf = int(read_num()); x = read_num(); y = read_num()
                p1 = (x, y) if absolute else (current[0] + x, current[1] + y)
                for pt in flatten_arc(current, rx, ry, rot, laf, sf, p1, step):
                    add_point(poly, pt)
                current = p1
                last_cubic_ctrl = None
                last_quad_ctrl = None

        elif c == "Z":
            add_point(poly, start)
            finish_poly()
            current = start
            last_cubic_ctrl = None
            last_quad_ctrl = None

        else:
            raise ValueError(f"Unsupported SVG path command: {cmd}")

    finish_poly()
    return polylines


# -------------------- SVG primitive helpers --------------------


def circle_polyline(cx: float, cy: float, r: float, step: float) -> Polyline:
    circumference = 2.0 * math.pi * r
    n = max(24, int(math.ceil(circumference / max(step, 0.1))))
    return [(cx + r * math.cos(2.0 * math.pi * i / n), cy + r * math.sin(2.0 * math.pi * i / n)) for i in range(n + 1)]


def ellipse_polyline(cx: float, cy: float, rx: float, ry: float, step: float) -> Polyline:
    circumference_approx = math.pi * (3 * (rx + ry) - math.sqrt((3 * rx + ry) * (rx + 3 * ry)))
    n = max(24, int(math.ceil(circumference_approx / max(step, 0.1))))
    return [(cx + rx * math.cos(2.0 * math.pi * i / n), cy + ry * math.sin(2.0 * math.pi * i / n)) for i in range(n + 1)]


def rect_polyline(x: float, y: float, w: float, h: float) -> Polyline:
    return [(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)]


def transformed_diameter_for_circle(cx: float, cy: float, r: float, matrix: Matrix) -> float:
    center = apply_mat(matrix, (cx, cy))
    x_edge = apply_mat(matrix, (cx + r, cy))
    y_edge = apply_mat(matrix, (cx, cy + r))
    # Handles uniform scale exactly and nonuniform scale approximately.
    return distance(x_edge, center) + distance(y_edge, center)


def layer_sort_for_name(layer_name: str, layer_order: int) -> Tuple[float, int]:
    match = _LAYER_NUM_RE.match(layer_name or "")
    if match:
        return (float(match.group(1)), layer_order)
    return (1_000_000.0 + layer_order, layer_order)


def get_layer_name(elem: ET.Element) -> str:
    # Inkscape layers often use inkscape:label; Illustrator usually uses id.
    for key, value in elem.attrib.items():
        if key.endswith("}label") or key == "inkscape:label":
            return value
    return elem.attrib.get("id", "")


def extract_svg_items(svg_file: Path, cfg: ConverterConfig) -> List[SvgItem]:
    tree = ET.parse(svg_file)
    root = tree.getroot()
    items: List[SvgItem] = []
    order_counter = 0
    layer_counter = 0

    def walk(elem: ET.Element, parent_style: Dict[str, str], parent_matrix: Matrix, layer_name: str, layer_sort: Tuple[float, int]) -> None:
        nonlocal order_counter, layer_counter
        tag = strip_ns(elem.tag)
        style = merge_style(parent_style, elem)
        matrix = mat_mul(parent_matrix, parse_transform(elem.attrib.get("transform")))

        if tag == "g":
            name = get_layer_name(elem)
            if name:
                layer_counter += 1
                layer_name = name
                layer_sort = layer_sort_for_name(name, layer_counter)

        if tag in {"svg", "g", "defs", "style", "metadata", "title", "desc"}:
            for child in list(elem):
                walk(child, style, matrix, layer_name, layer_sort)
            return

        order_counter += 1
        color = marker_color_from_style(style)

        try:
            if tag == "path":
                d = elem.attrib.get("d", "")
                polylines = [transform_polyline(p, matrix) for p in parse_path_d(d, cfg.sample_step_svg_units)]
                if polylines:
                    items.append(SvgItem("path", polylines, color=color, source_tag=tag, layer_name=layer_name, layer_sort=layer_sort, element_order=order_counter))

            elif tag == "line":
                x1 = parse_float(elem.attrib.get("x1")); y1 = parse_float(elem.attrib.get("y1"))
                x2 = parse_float(elem.attrib.get("x2")); y2 = parse_float(elem.attrib.get("y2"))
                poly = transform_polyline([(x1, y1), (x2, y2)], matrix)
                items.append(SvgItem("line", [poly], color=color, source_tag=tag, layer_name=layer_name, layer_sort=layer_sort, element_order=order_counter))

            elif tag == "polyline":
                pts = parse_points(elem.attrib.get("points", ""))
                if len(pts) >= 2:
                    items.append(SvgItem("polyline", [transform_polyline(pts, matrix)], color=color, source_tag=tag, layer_name=layer_name, layer_sort=layer_sort, element_order=order_counter))

            elif tag == "polygon":
                pts = parse_points(elem.attrib.get("points", ""))
                if len(pts) >= 2:
                    if pts[0] != pts[-1]:
                        pts.append(pts[0])
                    items.append(SvgItem("polygon", [transform_polyline(pts, matrix)], color=color, source_tag=tag, layer_name=layer_name, layer_sort=layer_sort, element_order=order_counter))

            elif tag == "circle":
                cx = parse_float(elem.attrib.get("cx")); cy = parse_float(elem.attrib.get("cy")); r = parse_float(elem.attrib.get("r"))
                if r > 0:
                    poly = transform_polyline(circle_polyline(cx, cy, r, cfg.sample_step_svg_units), matrix)
                    center = apply_mat(matrix, (cx, cy))
                    diameter = transformed_diameter_for_circle(cx, cy, r, matrix)
                    items.append(SvgItem("circle", [poly], center=center, diameter_svg_units=diameter, color=color, source_tag=tag, layer_name=layer_name, layer_sort=layer_sort, element_order=order_counter))

            elif tag == "ellipse":
                cx = parse_float(elem.attrib.get("cx")); cy = parse_float(elem.attrib.get("cy"))
                rx = parse_float(elem.attrib.get("rx")); ry = parse_float(elem.attrib.get("ry"))
                if rx > 0 and ry > 0:
                    poly = transform_polyline(ellipse_polyline(cx, cy, rx, ry, cfg.sample_step_svg_units), matrix)
                    center = apply_mat(matrix, (cx, cy))
                    items.append(SvgItem("ellipse", [poly], center=center, color=color, source_tag=tag, layer_name=layer_name, layer_sort=layer_sort, element_order=order_counter))

            elif tag == "rect":
                x = parse_float(elem.attrib.get("x")); y = parse_float(elem.attrib.get("y"))
                w = parse_float(elem.attrib.get("width")); h = parse_float(elem.attrib.get("height"))
                if w > 0 and h > 0:
                    poly = transform_polyline(rect_polyline(x, y, w, h), matrix)
                    center = apply_mat(matrix, (x + w / 2.0, y + h / 2.0))
                    items.append(SvgItem("rect", [poly], center=center, color=color, source_tag=tag, layer_name=layer_name, layer_sort=layer_sort, element_order=order_counter))

        except Exception as e:
            print(f"[WARNING] Skipping <{tag}> element because it could not be parsed: {e}")

        for child in list(elem):
            walk(child, style, matrix, layer_name, layer_sort)

    walk(root, {}, mat_identity(), "", (0.0, 0))
    return [item for item in items if item.polylines]


# -------------------- fitting / coordinate transform --------------------


def item_points_for_fitting(items: Sequence[SvgItem]) -> List[Point]:
    pts: List[Point] = []
    for item in items:
        if item.kind == "circle" and item.center is not None:
            # Pump marker circles are visual symbols, not traced geometry. Using
            # the center keeps marker size from bloating the fitted drawing.
            pts.append(item.center)
        else:
            for poly in item.polylines:
                pts.extend(poly)
    return pts


def raw_bbox_from_points(points: Sequence[Point]) -> Tuple[float, float, float, float]:
    if not points:
        raise ValueError("No drawable geometry or pump markers found in SVG.")
    xs = [pt[0] for pt in points]
    ys = [pt[1] for pt in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if math.isclose(min_x, max_x):
        min_x -= 0.5
        max_x += 0.5
    if math.isclose(min_y, max_y):
        min_y -= 0.5
        max_y += 0.5
    return min_x, max_x, min_y, max_y


def all_tool_offsets(cfg: ConverterConfig) -> List[ToolOffset]:
    return [cfg.stick_offset] + [cfg.pump_offsets[i] for i in sorted(cfg.pump_offsets)]


def compute_transform_metadata(items: Sequence[SvgItem], cfg: ConverterConfig) -> dict:
    min_x, max_x, min_y, max_y = raw_bbox_from_points(item_points_for_fitting(items))
    svg_w = max_x - min_x
    svg_h = max_y - min_y

    offsets = all_tool_offsets(cfg)
    min_off_x = min(o.x_mm for o in offsets)
    max_off_x = max(o.x_mm for o in offsets)
    min_off_y = min(o.y_mm for o in offsets)
    max_off_y = max(o.y_mm for o in offsets)

    usable_w = max(1.0, cfg.machine_width_mm - 2.0 * cfg.margin_mm)
    usable_h = max(1.0, cfg.machine_height_mm - 2.0 * cfg.margin_mm)
    target_w = min(cfg.plot_width_mm, usable_w)
    target_h = min(cfg.plot_height_mm, usable_h)

    # Reserve enough room for the full offset spread. With all offsets at 0 this
    # is exactly the normal 270 x 430 fit. Later, if a pump is physically offset
    # from the stick, the design scales down just enough that both the stick and
    # pump active points stay inside the working area.
    available_w_for_design = max(1.0, target_w - (max_off_x - min_off_x))
    available_h_for_design = max(1.0, target_h - (max_off_y - min_off_y))
    scale = min(available_w_for_design / svg_w, available_h_for_design / svg_h)

    final_w = svg_w * scale
    final_h = svg_h * scale

    # Center the combined envelope: design geometry plus all possible tool offsets.
    envelope_w = final_w + (max_off_x - min_off_x)
    envelope_h = final_h + (max_off_y - min_off_y)
    envelope_origin_x = (cfg.machine_width_mm - envelope_w) / 2.0 + cfg.offset_x_mm
    envelope_origin_y = (cfg.machine_height_mm - envelope_h) / 2.0 + cfg.offset_y_mm
    base_origin_x = envelope_origin_x - min_off_x
    base_origin_y = envelope_origin_y - min_off_y

    return {
        "svg_bbox": (min_x, max_x, min_y, max_y),
        "scale": scale,
        "origin_x_mm": base_origin_x,
        "origin_y_mm": base_origin_y,
        "final_w_mm": final_w,
        "final_h_mm": final_h,
        "envelope_w_mm": envelope_w,
        "envelope_h_mm": envelope_h,
        "min_offset_x_mm": min_off_x,
        "max_offset_x_mm": max_off_x,
        "min_offset_y_mm": min_off_y,
        "max_offset_y_mm": max_off_y,
    }


def transform_point(pt: Point, cfg: ConverterConfig, metadata: dict, offset: ToolOffset) -> Point:
    min_x, max_x, min_y, max_y = metadata["svg_bbox"]
    scale = metadata["scale"]
    origin_x = metadata["origin_x_mm"]
    origin_y = metadata["origin_y_mm"]
    print("POINT:")
    print(origin_x + (pt[0] - min_x) * scale)
    print(origin_y + (pt[1] - min_y) * scale)
    x = origin_x + (pt[0] - min_x) * scale + offset.x_mm
    if cfg.flip_y:
        y = origin_y + (max_y - pt[1]) * scale + offset.y_mm
    else:
        y = origin_y + (pt[1] - min_y) * scale + offset.y_mm
    print(x)
    print(y)
    print(f"FOR OFFSET {offset}")
    return x, y


def transform_polyline_to_machine(poly: Polyline, cfg: ConverterConfig, metadata: dict, offset: ToolOffset) -> Polyline:
    out: Polyline = []
    for pt in poly:
        new_pt = transform_point(pt, cfg, metadata, offset)
        if out and distance(out[-1], new_pt) < cfg.min_segment_mm:
            continue
        out.append(new_pt)
    return out


def is_pump_marker(item: SvgItem, cfg: ConverterConfig) -> Optional[int]:
    """
    A pump marker is any circle whose fill/stroke color matches one of the
    configured pump marker colors.

    Radius/diameter is intentionally ignored because SVG editors may rewrite
    circle sizes when transforms are applied. The circle is only a symbolic
    marker; its center point becomes the pump-drop location.
    """
    if item.kind != "circle" or item.center is None:
        return None
    if item.color is None:
        return None
    return PUMP_MARKER_COLORS.get(item.color)


def build_actions(items: Sequence[SvgItem], cfg: ConverterConfig, metadata: dict) -> Tuple[List[Action], dict]:
    actions: List[Action] = []
    pump_count = 0
    draw_count = 0
    unrecognized_circle_colors = 0

    for item in items:
        pump = is_pump_marker(item, cfg)
        if pump is not None:
            print(f"PUMP {pump}")
            print(f"OFFSET {cfg.pump_offsets[pump]}")
            point = transform_point(item.center, cfg, metadata, cfg.pump_offsets[pump])  # type: ignore[arg-type]
            actions.append(Action("pump", item.layer_sort, item.element_order, point=point, pump=pump, source_tag=item.source_tag, layer_name=item.layer_name))
            pump_count += 1
            continue

        # If a circle has a fill/stroke color, but that color is not one of the
        # recognized pump marker colors, warn and draw it as normal geometry.
        if item.kind == "circle":
            if item.color is None:
                unrecognized_circle_colors += 1
                print(
                    f"[WARNING] Circle on layer '{item.layer_name}' has no recognized fill/stroke color; "
                    f"treating it as drawable geometry."
                )
            elif item.color not in PUMP_MARKER_COLORS:
                unrecognized_circle_colors += 1
                print(
                    f"[WARNING] Circle on layer '{item.layer_name}' has unrecognized fill/stroke color "
                    f"{item.color!r}; treating it as drawable geometry."
                )

        transformed_polys: List[Polyline] = []
        for poly in item.polylines:
            out = transform_polyline_to_machine(poly, cfg, metadata, cfg.stick_offset)
            if len(out) >= 2:
                transformed_polys.append(out)
        if transformed_polys:
            actions.append(Action("draw", item.layer_sort, item.element_order, polylines=transformed_polys, source_tag=item.source_tag, layer_name=item.layer_name))
            draw_count += len(transformed_polys)

    actions.sort(key=lambda a: (a.layer_sort[0], a.layer_sort[1], a.element_order))
    stats = {
        "num_actions": len(actions),
        "num_draw_polylines": draw_count,
        "num_pump_markers": pump_count,
        "unrecognized_circle_colors": unrecognized_circle_colors,
    }
    return actions, stats


# -------------------- G-code output --------------------


def fmt_xy(pt: Point) -> str:
    return f"X{pt[0]:.3f} Y{pt[1]:.3f}"


def emit_stick_up(lines: List[str], cfg: ConverterConfig) -> None:
    lines.append("STICK_UP")
    lines.append(f"G4 P{cfg.stick_settle_delay_ms}")


def emit_stick_down(lines: List[str], cfg: ConverterConfig) -> None:
    lines.append("STICK_DOWN")
    lines.append(f"G4 P{cfg.stick_settle_delay_ms}")


def actions_to_klipper_gcode(actions: Sequence[Action], cfg: ConverterConfig, metadata: dict, stats: dict) -> List[str]:
    lines: List[str] = []

    lines.append("; Generated by svgKlipperConverter.py")
    lines.append("; XY + stick + pump file for Klipper")
    lines.append("; Draw geometry uses STICK_UP / STICK_DOWN with explicit G4 delays")
    lines.append("; Pump markers use DOSE PUMP=N SECONDS=... with explicit post-dose G4 delay")
    lines.append(f"; actions: {stats.get('num_actions')}")
    lines.append(f"; draw polylines: {stats.get('num_draw_polylines')}")
    lines.append(f"; pump markers: {stats.get('num_pump_markers')}")
    lines.append(f"; final design size: {metadata.get('final_w_mm'):.3f} x {metadata.get('final_h_mm'):.3f} mm")
    lines.append(f"; fitted envelope incl. tool offsets: {metadata.get('envelope_w_mm'):.3f} x {metadata.get('envelope_h_mm'):.3f} mm")
    lines.append(f"; base drawing origin: X{metadata.get('origin_x_mm'):.3f} Y{metadata.get('origin_y_mm'):.3f}")
    lines.append(f"; svg-to-mm scale: {metadata.get('scale'):.6f}")
    if stats.get("unrecognized_circle_colors"):
        lines.append(
            f"; warning: {stats.get('unrecognized_circle_colors')} circle(s) had unrecognized/no fill-stroke color and were drawn as geometry"
        )
    lines.append("")

    lines.append("G21 ; use millimeters")
    lines.append("G90 ; absolute positioning")
    if cfg.max_velocity_mm_s is not None or cfg.max_accel_mm_s2 is not None:
        parts = ["SET_VELOCITY_LIMIT"]
        if cfg.max_velocity_mm_s is not None:
            parts.append(f"VELOCITY={cfg.max_velocity_mm_s:.3f}")
        if cfg.max_accel_mm_s2 is not None:
            parts.append(f"ACCEL={cfg.max_accel_mm_s2:.3f}")
        lines.append(" ".join(parts))

    if cfg.include_home:
        lines.append("G28 ; home all axes")

    lines.append("")
    lines.append("; Begin actions")
    emit_stick_up(lines, cfg)

    current_layer = object()
    for i, action in enumerate(actions, start=1):
        if action.layer_name and action.layer_name != current_layer:
            lines.append("")
            lines.append(f"; Layer: {action.layer_name}")
            current_layer = action.layer_name

        if action.kind == "pump":
            if action.point is None or action.pump is None:
                continue
            lines.append(f"; Action {i}: pump marker, pump {action.pump}")
            emit_stick_up(lines, cfg)
            lines.append(f"G1 {fmt_xy(action.point)} F{cfg.travel_feed_mm_min:.0f}")
            lines.append(f"DOSE PUMP={action.pump} SECONDS={cfg.pump_pulse_seconds:.3f}")
            lines.append(f"G4 P{cfg.pump_settle_delay_ms}")

        elif action.kind == "draw":
            for j, polyline in enumerate(action.polylines, start=1):
                if len(polyline) < 2:
                    continue
                lines.append(f"; Action {i}: draw path {j}")
                emit_stick_up(lines, cfg)
                lines.append(f"G1 {fmt_xy(polyline[0])} F{cfg.travel_feed_mm_min:.0f}")
                emit_stick_down(lines, cfg)
                for pt in polyline[1:]:
                    lines.append(f"G1 {fmt_xy(pt)} F{cfg.draw_feed_mm_min:.0f}")
                emit_stick_up(lines, cfg)

    lines.append("")
    lines.append("; End actions")
    emit_stick_up(lines, cfg)
    if cfg.return_to_origin:
        lines.append(f"G1 X0.000 Y0.000 F{cfg.travel_feed_mm_min:.0f}")
    if cfg.include_m84:
        lines.append("M84 ; disable motors")
    return lines


def write_gcode(out_file: Path, lines: Iterable[str]) -> None:
    out_file.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


# -------------------- CLI --------------------


def none_or_float(value: str) -> Optional[float]:
    if value.strip().lower() in {"none", "off", "false", "no"}:
        return None
    return float(value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert an SVG into Klipper G-code with stick drawing and pump markers.")

    parser.add_argument("svg", type=Path, help="Input SVG file. Convert artwork to paths first when possible.")
    parser.add_argument("-o", "--out", type=Path, default=None, help="Output .gcode file. Default: input filename with .gcode extension.")

    parser.add_argument("--machine-width", type=float, default=270.0)
    parser.add_argument("--machine-height", type=float, default=430.0)
    parser.add_argument("--plot-width", type=float, default=270.0)
    parser.add_argument("--plot-height", type=float, default=430.0)
    parser.add_argument("--margin", type=float, default=0.0)
    parser.add_argument("--offset-x", type=float, default=0.0)
    parser.add_argument("--offset-y", type=float, default=0.0)

    parser.add_argument("--stick-offset-x", type=float, default=0.0)
    parser.add_argument("--stick-offset-y", type=float, default=0.0)
    for pump in range(4):
        parser.add_argument(f"--pump{pump}-offset-x", type=float, default=0.0)
        parser.add_argument(f"--pump{pump}-offset-y", type=float, default=0.0)

    parser.add_argument("--sample-step", type=float, default=4.0, help="Curve sampling step in SVG units. Smaller = smoother/more G-code.")
    parser.add_argument("--no-flip-y", action="store_true", help="Do not flip SVG Y coordinates into machine Y-up coordinates.")

    parser.add_argument("--draw-feed", type=float, default=5000.0, help="Draw/feed move speed in mm/min.")
    parser.add_argument("--travel-feed", type=float, default=10000.0, help="Travel move speed in mm/min.")
    parser.add_argument("--max-velocity", type=none_or_float, default=None, help="Optional Klipper velocity limit in mm/s. Use 'none' to omit.")
    parser.add_argument("--max-accel", type=none_or_float, default=None, help="Optional Klipper acceleration limit in mm/s^2. Use 'none' to omit.")

    parser.add_argument("--pump-pulse", type=float, default=PUMP_PULSE_SECONDS, help="Pump pulse duration in seconds. Default: 0.100")
    parser.add_argument("--pump-delay", type=int, default=PUMP_SETTLE_DELAY_MS, help="Delay after pump pulse in ms. Default: 250")
    parser.add_argument("--stick-delay", type=int, default=STICK_SETTLE_DELAY_MS, help="Delay after STICK_UP/STICK_DOWN in ms. Default: 150")
    parser.add_argument("--marker-diameter", type=float, default=PUMP_MARKER_DIAMETER_MM, help="SVG/design-space marker diameter. Default: 1.0")
    parser.add_argument("--marker-tolerance", type=float, default=PUMP_MARKER_DIAMETER_TOLERANCE_MM, help="Marker diameter tolerance in mm. Default: 0.15")

    parser.add_argument("--home", action="store_true", help="Put G28 at the top of the file.")
    parser.add_argument("--no-return-home", action="store_true", help="Do not move back to X0 Y0 at the end.")
    parser.add_argument("--disable-motors", action="store_true", help="Put M84 at the end of the file.")

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    svg_file: Path = args.svg
    if not svg_file.exists():
        raise SystemExit(f"Input SVG not found: {svg_file}")

    out_file: Path = args.out if args.out is not None else svg_file.with_suffix(".gcode")

    pump_offsets = {
        0: ToolOffset(args.pump0_offset_x, args.pump0_offset_y),
        1: ToolOffset(args.pump1_offset_x, args.pump1_offset_y),
        2: ToolOffset(args.pump2_offset_x, args.pump2_offset_y),
        3: ToolOffset(args.pump3_offset_x, args.pump3_offset_y),
    }

    cfg = ConverterConfig(
        machine_width_mm=args.machine_width,
        machine_height_mm=args.machine_height,
        plot_width_mm=args.plot_width,
        plot_height_mm=args.plot_height,
        margin_mm=args.margin,
        offset_x_mm=args.offset_x,
        offset_y_mm=args.offset_y,
        stick_offset=ToolOffset(args.stick_offset_x, args.stick_offset_y),
        sample_step_svg_units=args.sample_step,
        flip_y=not args.no_flip_y,
        draw_feed_mm_min=args.draw_feed,
        travel_feed_mm_min=args.travel_feed,
        max_velocity_mm_s=args.max_velocity,
        max_accel_mm_s2=args.max_accel,
        include_home=args.home,
        return_to_origin=not args.no_return_home,
        include_m84=args.disable_motors,
        pump_pulse_seconds=args.pump_pulse,
        pump_settle_delay_ms=args.pump_delay,
        stick_settle_delay_ms=args.stick_delay,
        pump_marker_diameter_mm=args.marker_diameter,
        pump_marker_tolerance_mm=args.marker_tolerance,
    )

    print(f"[INFO] Reading SVG: {svg_file}")
    items = extract_svg_items(svg_file, cfg)
    if not items:
        raise SystemExit("No drawable geometry or pump markers found. Convert artwork to paths/lines or use supported SVG primitives.")

    metadata = compute_transform_metadata(items, cfg)
    actions, stats = build_actions(items, cfg, metadata)
    if not actions:
        raise SystemExit("No actions found after parsing SVG.")

    gcode = actions_to_klipper_gcode(actions, cfg, metadata, stats)
    write_gcode(out_file, gcode)

    print(f"[INFO] Wrote: {out_file}")
    print(f"[INFO] Actions: {stats['num_actions']}")
    print(f"[INFO] Draw polylines: {stats['num_draw_polylines']}")
    print(f"[INFO] Pump markers: {stats['num_pump_markers']}")
    if stats["unrecognized_circle_colors"]:
        print(
            f"[WARNING] {stats['unrecognized_circle_colors']} circle(s) had unrecognized/no fill-stroke color "
            f"and were treated as drawable geometry."
        )
    print(f"[INFO] Final design size: {metadata['final_w_mm']:.3f} x {metadata['final_h_mm']:.3f} mm")
    print(f"[INFO] Fitted envelope incl. offsets: {metadata['envelope_w_mm']:.3f} x {metadata['envelope_h_mm']:.3f} mm")
    print(f"[INFO] Base drawing origin: X{metadata['origin_x_mm']:.3f} Y{metadata['origin_y_mm']:.3f}")
    print(f"[INFO] SVG-to-mm scale: {metadata['scale']:.6f}")


if __name__ == "__main__":
    main()
