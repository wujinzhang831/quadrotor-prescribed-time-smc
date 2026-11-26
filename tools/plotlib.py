# -*- coding: utf-8 -*-
"""Tiny publication-style plotting helpers on top of Pillow.

The package ships figures as PNG, generated without matplotlib so that the
repository has no third-party plotting dependency beyond Pillow.
"""
from __future__ import annotations

import math
import os

from PIL import Image, ImageDraw, ImageFont

FONT_REG = r"C:\Windows\Fonts\msyh.ttc"
FONT_BOLD = r"C:\Windows\Fonts\msyhbd.ttc"
FONT_MONO = r"C:\Windows\Fonts\times.ttf"

INK = (32, 32, 32)
GRID = (216, 216, 216)
FRAME = (90, 90, 90)
COLORS = [(200, 40, 40), (30, 130, 60), (40, 80, 190), (150, 60, 170),
          (210, 130, 20), (20, 150, 160), (120, 120, 120)]

_font_cache: dict = {}


def font(size: int, bold: bool = False):
    key = (size, bold)
    if key not in _font_cache:
        path = FONT_BOLD if bold else FONT_REG
        _font_cache[key] = ImageFont.truetype(path, int(round(size * 1.42)))
    return _font_cache[key]


def _nice_ticks(lo: float, hi: float, target: int = 5):
    if not (math.isfinite(lo) and math.isfinite(hi)) or hi <= lo:
        return [lo, hi]
    raw = (hi - lo) / max(target, 1)
    mag = 10 ** math.floor(math.log10(raw))
    for mult in (1, 2, 2.5, 5, 10):
        step = mult * mag
        if step >= raw:
            break
    start = math.ceil(lo / step) * step
    ticks = []
    v = start
    while v <= hi + 1e-12 * abs(hi):
        ticks.append(round(v, 10))
        v += step
    return ticks


def _fmt(v: float) -> str:
    if v == 0:
        return "0"
    a = abs(v)
    if a >= 1e4 or a < 1e-3:
        return f"{v:.1e}"
    if a >= 100:
        return f"{v:.0f}"
    if a >= 1:
        return f"{v:.1f}" if abs(v - round(v)) > 1e-9 else f"{v:.0f}"
    return f"{v:.2f}"


class Panel:
    """One axes area inside a canvas."""

    def __init__(self, cv, box, xlim, ylim, title="", xlabel="", ylabel="",
                 xlog=False, ylog=False, legend_loc="best", legend_ncol=1,
                 show_zero=False, fmt_x=None, fmt_y=None):
        self.cv = cv
        self.x0, self.y0, self.x1, self.y1 = box
        self.xlim = xlim
        self.ylim = ylim
        self.title = title
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.series = []
        self.legend_loc = legend_loc
        self.legend_ncol = legend_ncol
        self.show_zero = show_zero
        self.fmt_x = fmt_x or _fmt
        self.fmt_y = fmt_y or _fmt
        self.xticks = _nice_ticks(*xlim)
        self.yticks = _nice_ticks(*ylim)

    # -- data -------------------------------------------------------------
    def plot(self, xs, ys, color=COLORS[0], width=2, dash=False, label=None,
             marker=None, marker_every=0):
        self.series.append(dict(x=list(xs), y=list(ys), color=color, width=width,
                                dash=dash, label=label, marker=marker,
                                marker_every=marker_every))
        return self

    def axhline(self, y, color=COLORS[6], width=1, dash=True, label=None):
        self.series.append(dict(x=None, y=y, color=color, width=width, dash=dash,
                                label=label))
        return self

    def axvline(self, x, color=COLORS[6], width=1, dash=True, label=None):
        self.series.append(dict(x=x, y=None, color=color, width=width, dash=dash,
                                label=label))
        return self

    def note(self, x, y, text, color=INK, size=15):
        self.series.append(dict(text=text, x=x, y=y, color=color, size=size))
        return self

    # -- geometry ---------------------------------------------------------
    def px(self, x):
        lo, hi = self.xlim
        return self.x0 + (x - lo) / (hi - lo) * (self.x1 - self.x0)

    def py(self, y):
        lo, hi = self.ylim
        return self.y1 - (y - lo) / (hi - lo) * (self.y1 - self.y0)

    def _dashed(self, p0, p1, color, width, dash=7, gap=5):
        x0, y0 = p0
        x1, y1 = p1
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist == 0:
            return
        ux, uy = (x1 - x0) / dist, (y1 - y0) / dist
        pos = 0.0
        while pos < dist:
            end = min(pos + dash, dist)
            self.cv.draw.line([(x0 + ux * pos, y0 + uy * pos),
                               (x0 + ux * end, y0 + uy * end)], fill=color, width=width)
            pos = end + gap

    def _clip(self, xs, ys):
        """Clamp a polyline to the axes rectangle (cheap but robust)."""
        out = []
        lo_x, hi_x = self.xlim
        lo_y, hi_y = self.ylim
        for x, y in zip(xs, ys):
            if not math.isfinite(y) or x < lo_x or x > hi_x:
                continue
            out.append((self.px(x), self.py(max(min(y, hi_y), lo_y))))
        return out

    def render(self):
        cv = self.cv
        d = cv.draw
        # grid
        for tx in self.xticks:
            px = self.px(tx)
            d.line([(px, self.y0), (px, self.y1)], fill=GRID, width=1)
        for ty in self.yticks:
            py = self.py(ty)
            d.line([(self.x0, py), (self.x1, py)], fill=GRID, width=1)
        if self.show_zero:
            if self.ylim[0] < 0 < self.ylim[1]:
                d.line([(self.x0, self.py(0)), (self.x1, self.py(0))], fill=FRAME, width=1)
            if self.xlim[0] < 0 < self.xlim[1]:
                d.line([(self.px(0), self.y0), (self.px(0), self.y1)], fill=FRAME, width=1)
        # series
        for s in self.series:
            if "text" in s:
                cv.text(s["x"], s["y"], s["text"], color=s["color"], size=s["size"])
                continue
            if s["x"] is None:      # horizontal line
                p0, p1 = (self.x0, self.py(s["y"])), (self.x1, self.py(s["y"]))
            elif s["y"] is None:    # vertical line
                p0, p1 = (self.px(s["x"]), self.y0), (self.px(s["x"]), self.y1)
            else:
                pts = self._clip(s["x"], s["y"])
                if len(pts) < 2:
                    continue
                if s["dash"]:
                    for a, b in zip(pts[:-1], pts[1:]):
                        self._dashed(a, b, s["color"], s["width"])
                else:
                    d.line(pts, fill=s["color"], width=s["width"], joint="curve")
                if s["marker"] and s["marker_every"]:
                    for i in range(0, len(pts), s["marker_every"]):
                        cv.circle(pts[i], 5, fill=None, outline=s["color"], width=2)
                continue
            if s["dash"]:
                self._dashed(p0, p1, s["color"], s["width"], dash=8, gap=5)
            else:
                d.line([p0, p1], fill=s["color"], width=s["width"])
        # frame
        d.rectangle([self.x0, self.y0, self.x1, self.y1], outline=FRAME, width=2)
        # ticks + labels
        f_tick = font(17)
        for tx in self.xticks:
            px = self.px(tx)
            d.line([(px, self.y1), (px, self.y1 + 7)], fill=FRAME, width=2)
            cv.text(px, self.y1 + 10, self.fmt_x(tx), font_obj=f_tick,
                    color=INK, anchor="mt")
        for ty in self.yticks:
            py = self.py(ty)
            d.line([(self.x0 - 7, py), (self.x0, py)], fill=FRAME, width=2)
            cv.text(self.x0 - 12, py, self.fmt_y(ty), font_obj=f_tick,
                    color=INK, anchor="rm")
        if self.title:
            cv.text((self.x0 + self.x1) / 2, self.y0 - 14, self.title,
                    size=21, bold=True, anchor="mb", color=INK)
        if self.xlabel:
            cv.text((self.x0 + self.x1) / 2, self.y1 + 46, self.xlabel,
                    size=19, anchor="mt", color=INK)
        if self.ylabel:
            cv.text(self.x0 - 96, (self.y0 + self.y1) / 2, self.ylabel,
                    size=19, rotate=90, color=INK)
        self._legend(f_tick)

    def _legend(self, f_tick):
        items = [s for s in self.series if s.get("label")]
        if not items:
            return
        cv = self.cv
        fs = font(17)
        widths = [cv.text_size(s["label"], fs)[0] for s in items]
        line_w = 46
        col_w = max(widths) + line_w + 26
        ncol = min(self.legend_ncol, len(items))
        nrow = math.ceil(len(items) / ncol)
        box_w = ncol * col_w + 10
        box_h = nrow * 30 + 12
        if self.legend_loc == "upper right":
            lx, ly = self.x1 - box_w - 10, self.y0 + 10
        elif self.legend_loc == "lower left":
            lx, ly = self.x0 + 10, self.y1 - box_h - 10
        elif self.legend_loc == "upper left":
            lx, ly = self.x0 + 10, self.y0 + 10
        else:  # lower right
            lx, ly = self.x1 - box_w - 10, self.y1 - box_h - 10
        cv.draw.rectangle([lx, ly, lx + box_w, ly + box_h], fill=(255, 255, 255, 235),
                          outline=FRAME, width=1)
        for i, s in enumerate(items):
            col, row = divmod(i, nrow)
            iy = ly + 18 + row * 30
            ix = lx + 12 + col * col_w
            if s["x"] is None or s["y"] is None:
                if s["dash"]:
                    self._dashed((ix, iy), (ix + line_w - 12, iy), s["color"], s["width"], 6, 4)
                else:
                    cv.draw.line([(ix, iy), (ix + line_w - 12, iy)], fill=s["color"], width=s["width"])
            else:
                cv.draw.line([(ix, iy), (ix + line_w - 12, iy)], fill=s["color"], width=s["width"])
            cv.text(ix + line_w, iy, s["label"], font_obj=fs, color=INK, anchor="lm")


class Canvas:
    def __init__(self, width, height, bg=(255, 255, 255)):
        self.img = Image.new("RGB", (width, height), bg)
        self.draw = ImageDraw.Draw(self.img)
        self.w, self.h = width, height

    def text(self, x, y, s, size=19, bold=False, color=INK, anchor="mm",
             font_obj=None, rotate=0):
        f = font_obj or font(size, bold)
        if not rotate:
            self.draw.text((x, y), s, font=f, fill=color, anchor=anchor)
            return
        w, h = self.text_size(s, f)
        pad = 10
        tmp = Image.new("RGBA", (w + 2 * pad, h + 2 * pad), (0, 0, 0, 0))
        d2 = ImageDraw.Draw(tmp)
        d2.text((pad, pad), s, font=f, fill=color)
        tmp = tmp.rotate(rotate, expand=True)
        self.img.paste(tmp, (int(x - tmp.width / 2), int(y - tmp.height / 2)), tmp)

    def text_size(self, s, f):
        b = self.draw.textbbox((0, 0), s, font=f)
        return b[2] - b[0], b[3] - b[1]

    def circle(self, p, r, fill=None, outline=INK, width=2):
        self.draw.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], fill=fill,
                          outline=outline, width=width)

    def rect(self, box, fill=None, outline=INK, width=2, radius=10):
        self.draw.rounded_rectangle(box, radius=radius, fill=fill, outline=outline,
                                    width=width)

    def arrow(self, p0, p1, color=INK, width=2, head=12):
        d = self.draw
        d.line([p0, p1], fill=color, width=width)
        ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        for s in (+1, -1):
            a = ang + s * math.radians(24)
            d.line([p1, (p1[0] - head * math.cos(a), p1[1] - head * math.sin(a))],
                   fill=color, width=width)
        return self

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.img.save(path, optimize=True)
        return path


def grid_canvas(w, h, rows, cols, left=110, right=40, top=70, bottom=95,
                hgap=95, vgap=64):
    cv = Canvas(w, h)
    pw = (w - left - right - (cols - 1) * hgap) / cols
    ph = (h - top - bottom - (rows - 1) * vgap) / rows
    boxes = []
    for r in range(rows):
        row = []
        for c in range(cols):
            x0 = left + c * (pw + hgap)
            y0 = top + r * (ph + vgap)
            row.append((x0, y0, x0 + pw, y0 + ph))
        boxes.append(row)
    return cv, boxes
