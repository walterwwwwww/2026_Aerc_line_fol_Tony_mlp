#!/usr/bin/env python3
"""
make_shapes.py -- render the 10 tuning shapes to true-scale SVG + a manifest.

The SVG files are 1:1 with millimetres (viewBox == width/height in mm), so:
  * printing with "100% / actual size" (NOT "fit to page") gives a real template
  * a vinyl cutter / CNC / laser can consume the same file directly
  * the black stroke is exactly LINE_WIDTH_MM = 20 mm wide

For a 2 cm line the practical build is:
  * print the SVG on paper at actual size, or tile it across A4 sheets, then
  * lay glossy black vinyl tape / gaffer tape cut to 20 mm, or
  * paint with matte black on a light floor, or
  * use 20 mm black insulating tape (widely sold) straight onto a pale surface.

IMPORTANT (physics): the firmware thresholds a sensor as "on the line" when its
calibrated reflectance <= 6.0 on a 0..10 scale. That means the track surface
should be LIGHT (white / pale grey) and the line DARK (matte black), and both
must be consistent across the whole test area, otherwise every shape needs its
own calibration pass (BTN2 -> BTN3).

Run:  python make_shapes.py
"""

from __future__ import annotations

import json
import math
import os
import sys
from typing import List

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from shapes import (  # noqa: E402
    ARRAY_SPAN_MM,
    LINE_HALF_MM,
    LINE_WIDTH_MM,
    SENSOR_COUNT,
    SENSOR_PITCH_MM,
    Shape,
    build_shapes,
)

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "shapes")


# --------------------------------------------------------------------------
#  SVG rendering
# --------------------------------------------------------------------------
def shape_to_svg(sh: Shape, scale: float = 1.0) -> str:
    """
    Render the shape's centre-line as a 20 mm black stroke.

    scale: 1.0 => 1 SVG user unit = 1 mm (true size).
    """
    w = sh.width * scale
    h = sh.height * scale
    half = LINE_HALF_MM * scale

    parts: List[str] = []
    parts.append(
        f'<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{w:.2f}mm" height="{h:.2f}mm" '
        f'viewBox="0 0 {w:.2f} {h:.2f}">'
    )
    # title / metadata
    parts.append(f"<title>{_esc(sh.name)}</title>")
    parts.append(
        f"<desc>{_esc(sh.tunes)} | line width {LINE_WIDTH_MM:.0f} mm | "
        f"path {sh.total_path_mm:.0f} mm</desc>"
    )
    parts.append(
        '<rect x="0" y="0" width="100%" height="100%" fill="#ffffff"/>'
    )

    # centre-line path, drawn as a thick black stroke => the real 2 cm line
    d = " ".join(
        f"{'M' if i == 0 else 'L'} {r.x0*scale:.3f} {r.y0*scale:.3f} "
        f"L {r.x1*scale:.3f} {r.y1*scale:.3f}"
        for i, r in enumerate(sh.runs)
    )
    parts.append(
        f'<path d="{d}" fill="none" stroke="#000000" '
        f'stroke-width="{LINE_WIDTH_MM*scale:.3f}" '
        f'stroke-linecap="butt" stroke-linejoin="miter"/>'
    )

    # thin dashed centre-line guide for alignment/debug (does not affect the line)
    parts.append(
        f'<path d="{d}" fill="none" stroke="#ff0000" stroke-width="{0.4*scale:.2f}" '
        f'stroke-dasharray="{10*scale:.1f},{10*scale:.1f}" opacity="0.55"/>'
    )

    # 100 mm calibration ruler along the bottom, so the print can be verified
    _ruler(parts, sh, scale)

    # dimension caption
    parts.append(
        f'<text x="{6*scale:.1f}" y="{12*scale:.1f}" font-family="monospace" '
        f'font-size="{5*scale:.1f}" fill="#333">{_esc(sh.name)}</text>'
    )

    parts.append("</svg>")
    return "\n".join(parts)


def _ruler(parts: List[str], sh: Shape, scale: float) -> None:
    """A 100 mm reference bar with 10 mm ticks -- verify the print is true size."""
    y = sh.height - 12.0
    x = 6.0
    parts.append(
        f'<line x1="{x*scale:.2f}" y1="{y*scale:.2f}" '
        f'x2="{(x+100)*scale:.2f}" y2="{y*scale:.2f}" '
        f'stroke="#0066cc" stroke-width="{0.6*scale:.2f}"/>'
    )
    for i in range(11):
        tx = x + i * 10.0
        th = 5.0 if i % 5 == 0 else 2.5
        parts.append(
            f'<line x1="{tx*scale:.2f}" y1="{y*scale:.2f}" '
            f'x2="{tx*scale:.2f}" y2="{(y-th)*scale:.2f}" '
            f'stroke="#0066cc" stroke-width="{0.6*scale:.2f}"/>'
        )
    parts.append(
        f'<text x="{(x+102)*scale:.2f}" y="{(y+1.5)*scale:.2f}" '
        f'font-family="monospace" font-size="{4*scale:.1f}" fill="#0066cc">'
        f'100 mm check</text>'
    )


def _esc(s: str) -> str:
    return (
        s.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


# --------------------------------------------------------------------------
#  DXF (optional, for CNC / laser / vinyl)
# --------------------------------------------------------------------------
def shape_to_dxf(sh: Shape) -> str:
    """
    Minimal DXF R12 with the centre-line as LINE entities on layer 'LINE_CENTER'
    and the 20 mm outline offset as polylines on layer 'LINE_EDGE'.

    Most cutters want an OUTLINE, not a centre-line, so we emit the two offset
    boundaries of the 2 cm line (the shape's own mitred offset).
    """
    out: List[str] = []

    def pair(code: int, val: str) -> None:
        out.append(str(code))
        out.append(str(val))

    # header
    pair(0, "SECTION"); pair(2, "HEADER")
    pair(9, "$ACADVER"); pair(1, "AC1009")
    pair(9, "$INSUNITS"); pair(70, 4)  # 4 = millimetres
    pair(0, "ENDSEC")

    # tables: two layers
    pair(0, "SECTION"); pair(2, "TABLES")
    pair(0, "TABLE"); pair(2, "LAYER"); pair(70, 2)
    for name, color in (("LINE_CENTER", 1), ("LINE_EDGE", 7)):
        pair(0, "LAYER"); pair(2, name); pair(70, 0); pair(62, color); pair(6, "CONTINUOUS")
    pair(0, "ENDTAB"); pair(0, "ENDSEC")

    # entities
    pair(0, "SECTION"); pair(2, "ENTITIES")

    def line(x0, y0, x1, y1, layer):
        pair(0, "LINE"); pair(8, layer)
        pair(10, f"{x0:.4f}"); pair(20, f"{-y0:.4f}"); pair(30, "0.0")
        pair(11, f"{x1:.4f}"); pair(21, f"{-y1:.4f}"); pair(31, "0.0")

    for r in sh.runs:
        line(r.x0, r.y0, r.x1, r.y1, "LINE_CENTER")

    # offset outlines (left and right of travel) as mitred polylines
    for side in (+1.0, -1.0):
        pts = []
        for r in sh.runs:
            nx, ny = r.normal()
            pts.append((r.x0 + nx * LINE_HALF_MM * side, r.y0 + ny * LINE_HALF_MM * side))
            pts.append((r.x1 + nx * LINE_HALF_MM * side, r.y1 + ny * LINE_HALF_MM * side))
        for i in range(len(pts) - 1):
            line(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], "LINE_EDGE")

    pair(0, "ENDSEC"); pair(0, "EOF")
    return "\n".join(out)


# --------------------------------------------------------------------------
#  Manifest
# --------------------------------------------------------------------------
def build_manifest(shapes: List[Shape]) -> dict:
    return {
        "format": "linfol-ml-tuning-shapes-v1",
        "line_width_mm": LINE_WIDTH_MM,
        "sensor": {
            "count": SENSOR_COUNT,
            "pitch_mm": SENSOR_PITCH_MM,
            "array_span_mm": ARRAY_SPAN_MM,
        },
        "surface": {
            "line": "matte black, 20 mm wide",
            "background": "light / white, uniform",
            "why": "firmware marks a sensor as on-line when calibrated reflectance <= 6.0 of 0..10",
        },
        "shapes": [
            {
                "order": i + 1,
                "slug": sh.slug,
                "name": sh.name,
                "canvas_mm": [round(sh.width, 1), round(sh.height, 1)],
                "path_length_mm": round(sh.total_path_mm, 1),
                "runs": len(sh.runs),
                "difficulty": sh.difficulty,
                "tunes": sh.tunes,
                "how_to_use": sh.how,
                "notes": sh.notes,
                "tags": sh.tags,
                "files": {
                    "svg": f"shapes/{sh.slug}.svg",
                    "dxf": f"shapes/{sh.slug}.dxf",
                },
            }
            for i, sh in enumerate(shapes)
        ],
    }


def main() -> int:
    os.makedirs(OUT_DIR, exist_ok=True)
    shapes = build_shapes()

    for sh in shapes:
        with open(os.path.join(OUT_DIR, f"{sh.slug}.svg"), "w", encoding="utf-8") as f:
            f.write(shape_to_svg(sh))
        with open(os.path.join(OUT_DIR, f"{sh.slug}.dxf"), "w", encoding="utf-8") as f:
            f.write(shape_to_dxf(sh))

    manifest = build_manifest(shapes)
    with open(os.path.join(OUT_DIR, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)

    # an index page that shows all 10 at reduced scale, for a quick overview
    with open(os.path.join(OUT_DIR, "index.html"), "w", encoding="utf-8") as f:
        f.write(_index_html(shapes))

    print(f"Wrote {len(shapes)} shapes to {OUT_DIR}")
    print(f"  SVG + DXF per shape, plus manifest.json and index.html")
    print(f"  line width: {LINE_WIDTH_MM:.0f} mm   sensor pitch: {SENSOR_PITCH_MM:.0f} mm")
    return 0


def _index_html(shapes: List[Shape]) -> str:
    rows = []
    for i, sh in enumerate(shapes, 1):
        # scale each to fit a 520 px wide preview box
        sc = min(520.0 / sh.width, 320.0 / sh.height)
        rows.append(
            f'<div class="card"><h3>{i}. {_esc(sh.name)}</h3>'
            f'<div class="meta">{sh.total_path_mm:.0f} mm path &middot; '
            f'{len(sh.runs)} runs &middot; canvas {sh.width:.0f}&times;{sh.height:.0f} mm</div>'
            f'<div class="svgbox">{shape_to_svg_scaled(sh, sc)}</div>'
            f'<p><b>Tunes:</b> {_esc(sh.tunes)}</p>'
            f'<p><b>How:</b> {_esc(sh.how)}</p>'
            f'<p class="files"><a href="{sh.slug}.svg">SVG</a> &middot; '
            f'<a href="{sh.slug}.dxf">DXF</a></p></div>'
        )
    return (
        "<!doctype html><html><head><meta charset='utf-8'>"
        "<title>LineFollower - 10 tuning shapes</title><style>"
        "body{font-family:system-ui,sans-serif;margin:24px;background:#f6f7f9;color:#16181d}"
        "h1{font-size:20px}.card{background:#fff;border:1px solid #dfe3e8;border-radius:10px;"
        "padding:16px;margin:16px 0;max-width:820px}"
        ".card h3{margin:0 0 4px;font-size:15px}.meta{color:#5b6472;font-size:12px;margin-bottom:8px}"
        ".svgbox{background:#fff;border:1px dashed #cfd6de;border-radius:6px;padding:8px;overflow:auto}"
        "p{font-size:13px;line-height:1.45;margin:6px 0}.files a{color:#0b6bcb}"
        "</style></head><body><h1>LineFollower &mdash; 10 physical tuning shapes "
        "(20 mm line)</h1>"
        "<p>Preview only. SVG files are true 1:1 millimetre scale &mdash; print at "
        "<b>100% / actual size</b>.</p>" + "".join(rows) + "</body></html>"
    )


def shape_to_svg_scaled(sh: Shape, scale: float) -> str:
    svg = shape_to_svg(sh, scale=scale)
    # strip the physical mm sizing so it behaves as a responsive preview
    import re

    svg = re.sub(r'width="[\d.]+mm"\s+height="[\d.]+mm"', "", svg, count=1)
    return svg


if __name__ == "__main__":
    raise SystemExit(main())
