#!/usr/bin/env python3
"""Procedural textures for the warehouse (no dependencies, deterministic).

Plain grey surfaces give the ZED's visual odometry nothing to track. Each surface gets its own image
(no repeated tile, which would add more look-alike places to the shelves' ones).

Walls (inner faces, image = the face seen from inside, top row = top of the wall):
  - concrete blocks 0.40 x 0.20 m in running bond, each with its own shade, mortar joints,
    grain, large dirt stains and grime near the floor
  - painted base band with hazard stripes in places, forklift scuffs
  - steel columns every ~6 m with base plates, a cable tray with brackets, a pipe with flanges
  - panels: fire cabinets, electrical boxes, safety and information signs, roll-up doors
Floor (inner area, image columns along world Y, rows from world X max (top) to X min):
  - polished concrete slabs with saw-cut joints every 6 m, each slab with its own shade, stains,
    oil spots, tyre marks
  - markings following the layout: yellow lines around the shelf lines and zone 4, dashed aisle
    centre lines, direction arrows, stop bars, a pedestrian walkway with zebra crossings
Roof (underside of the deck, same mapping as the floor): corrugated steel sheet, joints, fasteners
  on the purlins, stains, skylight panels
Cartons: atlas of 8 side and 8 top face variants (kraft or white board, worn edges, tape, seams,
  hand holes, printed blocks, "this way up" arrows)
Pallet labels: atlas of GS1-style logistics labels, one per pallet, with one ITF-14 barcode (the
  GS1 case code printed on shipping cartons), valid check digit and human-readable digits. In
  warehouses 1D GS1 barcodes are the standard (SSCC in GS1-128 on pallet labels, ITF-14 on cases);
  QR codes are rare on logistics labels. GS1-128 itself is not drawn (its Code 128 table is not
  reproduced)

Used by scripts/gen_warehouse_layout.py; run alone to regenerate only the textures:
  scripts/gen_warehouse_textures.py
"""
import random
import struct
import zlib
from pathlib import Path

PPM = 48          # walls, pixels per metre (~2 cm/px; the ZED at VGA sees ~1.3 cm/px at 5 m)
FLOOR_PPM = 40    # floor, 2.5 cm/px (seen at grazing angles, 68 x 28 m)
BLOCK_W, BLOCK_H = 0.40, 0.20
CONCRETE = (148, 147, 141)


def write_png(path, w, h, rows):
    raw = b"".join(b"\x00" + bytes(r) for r in rows)

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))

    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


class Wall:
    ppm = PPM

    def __init__(self, length, height, seed):
        self.rng = random.Random(seed)
        self.w, self.h = round(length * self.ppm), round(height * self.ppm)
        self.length, self.height = length, height
        self.rows = [bytearray(3 * self.w) for _ in range(self.h)]

    # --- helpers in metres: x from the left end, z from the floor --------------------------
    def px(self, x):
        return max(0, min(max(self.w, self.h), round(x * self.ppm)))

    def blob(self, xc, zc, rx, rz, f):
        # darken an ellipse (stains, oil spots), softer towards the edge
        for r in range(max(0, self.h - self.px(zc + rz)), min(self.h, self.h - self.px(zc - rz))):
            z = (self.h - 0.5 - r) / self.ppm
            dz = (z - zc) / rz
            if abs(dz) >= 1:
                continue
            half = rx * (1 - dz * dz) ** 0.5
            row = self.rows[r]
            for x in range(min(self.px(xc - half), self.w), min(self.px(xc + half), self.w)):
                d = ((x / self.ppm - xc) / rx) ** 2 + dz * dz
                k = 1 - (1 - f) * (1 - d)
                for i in range(3 * x, 3 * x + 3):
                    row[i] = int(row[i] * k)

    def rect(self, x0, z0, x1, z1, color):
        a, b = min(self.px(x0), self.w), min(self.px(x1), self.w)
        if b <= a:
            return
        fill = bytes(color) * (b - a)
        for r in range(max(0, self.h - self.px(z1)), min(self.h, self.h - self.px(z0))):
            self.rows[r][3 * a:3 * b] = fill

    def frame(self, x0, z0, x1, z1, color, t=0.02):
        self.rect(x0, z0, x1, z0 + t, color)
        self.rect(x0, z1 - t, x1, z1, color)
        self.rect(x0, z0, x0 + t, z1, color)
        self.rect(x1 - t, z0, x1, z1, color)

    def shade(self, x0, z0, x1, z1, f):
        a, b = min(self.px(x0), self.w), min(self.px(x1), self.w)
        for r in range(max(0, self.h - self.px(z1)), min(self.h, self.h - self.px(z0))):
            row = self.rows[r]
            for i in range(3 * a, 3 * b):
                row[i] = min(255, int(row[i] * f))

    # --- layers --------------------------------------------------------------------------------
    def concrete(self):
        rng = self.rng
        # dirt stains: coarse random grid (1.5 m) linearly interpolated
        step = 1.5 * self.ppm
        gw, gh = int(self.w / step) + 2, int(self.h / step) + 2
        grid = [[rng.uniform(-1, 1) for _ in range(gw)] for _ in range(gh)]
        cols = []
        for g in grid:
            line = []
            for x in range(self.w):
                k, f = divmod(x / step, 1)
                k = int(k)
                line.append(g[k] * (1 - f) + g[k + 1] * f)
            cols.append(line)
        bw, bh = BLOCK_W * self.ppm, BLOCK_H * self.ppm
        nbr, nbc = int(self.h / bh) + 2, int(self.w / bw) + 3
        blocks = [[rng.gauss(0, 1) for _ in range(nbc)] for _ in range(nbr)]
        for r in range(self.h):
            z = self.h - 1 - r                       # pixel height from the floor
            k, f = divmod(z / step, 1)
            k = int(k)
            s0, s1 = cols[k], cols[k + 1]
            br = int(z / bh)
            mortar_row = z - br * bh < 1
            offset = bw / 2 if br % 2 else 0.0
            brow = blocks[br]
            grime = 1 - 0.3 * max(0.0, 1 - z / (0.7 * self.ppm))
            row = self.rows[r]
            for x in range(self.w):
                bx = (x + offset) / bw
                bc = int(bx)
                v = (1 + 0.06 * brow[bc]) * (1 + 0.14 * (s0[x] * (1 - f) + s1[x] * f)) * grime
                if mortar_row or bx - bc < 1 / bw:
                    v *= 0.72
                v += rng.uniform(-0.05, 0.05)
                i = 3 * x
                row[i] = min(255, max(0, int(CONCRETE[0] * v)))
                row[i + 1] = min(255, max(0, int(CONCRETE[1] * v)))
                row[i + 2] = min(255, max(0, int(CONCRETE[2] * v)))

    def base_band(self):
        rng = self.rng
        self.rect(0, 0, self.length, 0.25, (205, 170, 40))
        self.rect(0, 0.25, self.length, 0.27, (60, 55, 40))
        x = rng.uniform(1, 6)
        while x < self.length - 3:                    # hazard stripes in a few segments
            seg = rng.uniform(1.5, 3.5)
            for k in range(int(seg / 0.3)):
                for j in range(6):                    # diagonal stripe as small steps
                    xs = x + k * 0.3 + j * 0.025
                    self.rect(xs, j * 0.042, xs + 0.12, (j + 1) * 0.042, (25, 25, 25))
            x += seg + rng.uniform(6, 14)
        for _ in range(int(self.length / 2.5)):       # forklift scuffs
            x0, z0 = rng.uniform(0, self.length - 1), rng.uniform(0.05, 0.9)
            self.shade(x0, z0, x0 + rng.uniform(0.2, 1.2), z0 + rng.uniform(0.02, 0.06), 0.55)

    def columns(self):
        rng, spans = self.rng, []
        x = rng.uniform(2, 4)
        while x < self.length - 1:
            c = (60 + rng.randint(-10, 10), 80, 115)
            self.rect(x, 0, x + 0.3, self.height, c)
            self.rect(x, 0, x + 0.03, self.height, (35, 45, 65))
            self.rect(x + 0.27, 0, x + 0.3, self.height, (35, 45, 65))
            self.rect(x - 0.08, 0, x + 0.38, 0.12, (90, 95, 100))           # base plate
            for z in (0.3, 4.0, 7.2):                                         # bolted plates
                self.rect(x + 0.06, z, x + 0.24, z + 0.18, (45, 55, 80))
                for dx in (0.09, 0.19):
                    self.rect(x + dx, z + 0.06, x + dx + 0.03, z + 0.09, (200, 200, 190))
            spans.append((x - 0.2, x + 0.5))
            x += rng.uniform(5.5, 6.5)
        return spans

    def services(self):
        rng = self.rng
        z = rng.uniform(5.2, 5.8)                                             # cable tray
        self.rect(0, z, self.length, z + 0.12, (135, 138, 140))
        self.rect(0, z, self.length, z + 0.02, (70, 72, 75))
        x = rng.uniform(0, 1)
        while x < self.length:
            self.rect(x, z - 0.25, x + 0.04, z, (80, 80, 85))
            x += 1.5
        zp = rng.uniform(6.3, 6.9)                                            # pipe with flanges
        pc = rng.choice([(170, 40, 35), (110, 115, 120), (40, 110, 60)])
        for k, f in enumerate((0.65, 0.85, 1.0, 1.1, 1.0, 0.85, 0.65)):       # cylinder shading
            self.rect(0, zp + k * 0.02, self.length, zp + (k + 1) * 0.02,
                      tuple(min(255, int(c * f)) for c in pc))
        x = rng.uniform(0, 3)
        while x < self.length:
            self.rect(x, zp - 0.02, x + 0.06, zp + 0.16, tuple(int(c * 0.6) for c in pc))
            x += rng.uniform(2.5, 4)

    def panels(self, spans, doors):
        rng = self.rng
        busy = list(spans)

        def free(x0, x1):
            return all(x1 < a or x0 > b for a, b in busy)

        for _ in range(doors):                                                 # roll-up doors
            for _ in range(50):
                x0 = rng.uniform(2, self.length - 6)
                if free(x0, x0 + 4):
                    break
            else:
                continue
            self.rect(x0 - 0.12, 0, x0 + 4.12, 4.6, (200, 170, 40))
            self.rect(x0, 0, x0 + 4, 4.5, (150, 152, 155))
            for k in range(int(4.5 / 0.15)):
                self.rect(x0, k * 0.15, x0 + 4, k * 0.15 + 0.02, (105, 107, 110))
            self.rect(x0 + 1.7, 0.9, x0 + 2.3, 0.97, (60, 60, 60))           # handle
            busy.append((x0 - 0.3, x0 + 4.3))
        n = int(self.length / 3.5)
        for _ in range(n):
            kind = rng.choice(["fire", "electric", "electric", "safety", "info", "info"])
            wpan, hpan = {"fire": (0.5, 0.8), "electric": (0.6, 0.8), "safety": (0.4, 0.4),
                          "info": (0.7, 0.45)}[kind]
            for _ in range(50):
                x0 = rng.uniform(0.5, self.length - wpan - 0.5)
                if free(x0, x0 + wpan):
                    break
            else:
                continue
            z0 = rng.uniform(1.0, 1.6) if kind in ("fire", "electric") else rng.uniform(1.6, 2.6)
            x1, z1 = x0 + wpan, z0 + hpan
            if kind == "fire":
                self.rect(x0, z0, x1, z1, (185, 30, 30))
                self.frame(x0 + 0.05, z0 + 0.05, x1 - 0.05, z1 - 0.05, (240, 240, 240), 0.025)
                cx, cz = (x0 + x1) / 2, (z0 + z1) / 2
                self.rect(cx - 0.12, cz - 0.03, cx + 0.12, cz + 0.03, (240, 240, 240))
                self.rect(cx - 0.03, cz - 0.12, cx + 0.03, cz + 0.12, (240, 240, 240))
            elif kind == "electric":
                g = rng.randint(160, 190)
                self.rect(x0, z0, x1, z1, (g, g, g + 5))
                self.frame(x0, z0, x1, z1, (70, 70, 75), 0.025)
                self.rect(x1 - 0.06, z0 + 0.3, x1 - 0.03, z0 + 0.45, (60, 60, 60))   # latch
                for k, lc in enumerate([(220, 40, 30), (40, 200, 60), (230, 190, 40)]):
                    self.rect(x0 + 0.08 + k * 0.08, z1 - 0.12, x0 + 0.13 + k * 0.08, z1 - 0.07, lc)
                self.rect(x0 + 0.1, z0 + 0.1, x0 + 0.35, z0 + 0.2, (240, 210, 40))     # warning label
                self.rect(x0 + 0.1, z1, x0 + 0.14, z1 + rng.uniform(1, 3), (50, 50, 50))  # conduit
            elif kind == "safety":
                self.rect(x0, z0, x1, z1, (20, 135, 60))
                self.rect(x0 + 0.08, z0 + 0.17, x0 + 0.24, z0 + 0.23, (245, 245, 245))   # arrow
                for j in range(5):
                    self.rect(x0 + 0.24, z0 + 0.1 + j * 0.03, x0 + 0.24 + (j + 1) * 0.02,
                              z0 + 0.13 + j * 0.03, (245, 245, 245))
            else:
                self.rect(x0, z0, x1, z1, rng.choice([(30, 70, 150), (240, 240, 235), (230, 190, 30)]))
                ink = (245, 245, 245) if self.rows[self.h - self.px(z0) - 1][3 * self.px(x0)] < 100 \
                    else (25, 25, 25)
                for k in range(4):                                             # text-like lines
                    lw = rng.uniform(0.25, 0.55)
                    self.rect(x0 + 0.07, z1 - 0.11 - k * 0.08, x0 + 0.07 + lw, z1 - 0.08 - k * 0.08, ink)
            busy.append((x0 - 0.1, x1 + 0.1))


class Floor(Wall):
    """Floor seen from above: drawing x = world Y + half_l (columns), z = world X + half_w (rows,
    bottom = world X min), i.e. u = (Y + half_l) / length, v = (X + half_w) / width."""
    ppm = FLOOR_PPM
    base = (128, 128, 124)

    def __init__(self, length, width, seed):
        super().__init__(length, width, seed)
        self.hl, self.hw = length / 2, width / 2

    def area(self, xa, xb, ya, yb, color):
        """Rectangle in world coordinates (X from xa to xb, Y from ya to yb)."""
        self.rect(ya + self.hl, xa + self.hw, yb + self.hl, xb + self.hw, color)

    def outline(self, xa, xb, ya, yb, color, t=0.10, skip=()):
        if "xa" not in skip:
            self.area(xa, xa + t, ya, yb, color)
        if "xb" not in skip:
            self.area(xb - t, xb, ya, yb, color)
        self.area(xa, xb, ya, ya + t, color)
        self.area(xa, xb, yb - t, yb, color)

    def concrete(self):
        rng = self.rng
        step = 2.0 * self.ppm                     # stains, coarse grid interpolated
        gw, gh = int(self.w / step) + 2, int(self.h / step) + 2
        grid = [[rng.uniform(-1, 1) for _ in range(gw)] for _ in range(gh)]
        cols = []
        for g in grid:
            line = []
            for x in range(self.w):
                k, f = divmod(x / step, 1)
                k = int(k)
                line.append(g[k] * (1 - f) + g[k + 1] * f)
            cols.append(line)
        slab = 6.0 * self.ppm                     # saw-cut joints every 6 m, from the centre
        ox, oz = (self.hl % 6.0) * self.ppm, (self.hw % 6.0) * self.ppm
        shades = [[rng.gauss(0, 1) for _ in range(int(self.w / slab) + 3)]
                  for _ in range(int(self.h / slab) + 3)]
        for r in range(self.h):
            z = self.h - 1 - r
            k, f = divmod(z / step, 1)
            k = int(k)
            s0, s1 = cols[k], cols[k + 1]
            sz = (z - oz) / slab + 1
            joint_row = (z - oz) % slab < 1
            srow = shades[int(sz)]
            row = self.rows[r]
            for x in range(self.w):
                sx = (x - ox) / slab + 1
                v = (1 + 0.05 * srow[int(sx)]) * (1 + 0.12 * (s0[x] * (1 - f) + s1[x] * f))
                if joint_row or (x - ox) % slab < 1:
                    v *= 0.55
                v += rng.uniform(-0.07, 0.07)
                if rng.random() < 0.004:              # aggregate speckles
                    v *= rng.uniform(0.6, 1.3)
                i = 3 * x
                row[i] = min(255, max(0, int(self.base[0] * v)))
                row[i + 1] = min(255, max(0, int(self.base[1] * v)))
                row[i + 2] = min(255, max(0, int(self.base[2] * v)))

    def wear(self, aisles):
        rng = self.rng
        for _ in range(int(self.hl * self.hw / 6)):            # oil spots and stains
            self.blob(rng.uniform(0, 2 * self.hl), rng.uniform(0, 2 * self.hw),
                      rng.uniform(0.1, 0.5), rng.uniform(0.1, 0.4), rng.uniform(0.55, 0.85))
        for xa, xb in aisles:                                   # tyre marks along the aisles
            for _ in range(6):
                x = rng.uniform(xa + 0.5, xb - 0.5)
                y = rng.uniform(-self.hl + 2, self.hl - 12)
                ln = rng.uniform(3, 10)
                drift = rng.uniform(-0.15, 0.15)
                k = 0.0
                while k < ln:
                    xx = x + drift * k + 0.3 * ((k / ln) ** 2) * rng.choice((-1, 1))
                    for off in (0.0, rng.uniform(0.9, 1.1)):    # two wheels
                        self.shade(y + k + self.hl, xx + off + self.hw,
                                   y + k + 0.06 + self.hl, xx + off + 0.08 + self.hw, 0.82)
                    k += 0.05

    def arrow(self, xc, yc, direction, color=(235, 235, 230)):
        """Floor arrow along Y (direction +1 / -1), 1.6 m long."""
        self.area(xc - 0.08, xc + 0.08, yc - 0.8 * direction if direction > 0 else yc,
                  yc if direction > 0 else yc + 0.8, color)
        tip = yc + 0.8 * direction
        for j in range(8):                                       # triangular head in steps
            hw = 0.35 * (1 - j / 8)
            y0 = yc + direction * j * 0.1
            self.area(xc - hw, xc + hw, min(y0, y0 + 0.1 * direction),
                      max(y0, y0 + 0.1 * direction), color)
        return tip

    def markings(self, lines, zone4, aisles, y_end):
        yellow, white, green = (215, 180, 40), (232, 232, 226), (40, 130, 70)
        for xa, xb in lines:                                     # around the shelf lines
            at_wall = xa <= -self.hw + 0.01
            self.outline(xa - 0.15, xb + 0.15, -y_end - 0.15, y_end + 0.15, yellow,
                         skip=("xa",) if at_wall else ())
        xa, xb, ya, yb = zone4                                   # zone 4: outline + hatched ends
        self.outline(xa - 0.15, min(xb, self.hw) - 0.01, ya - 0.15, yb + 0.15, yellow,
                     skip=("xb",) if xb >= self.hw - 0.01 else ())
        for y0 in (ya - 1.4, yb + 0.25):
            self.area(xa - 0.15, self.hw, y0, y0 + 1.15, yellow)
            k = 0
            while k * 0.3 < 1.15:
                for j in range(6):
                    xs = xa - 0.15 + j * ((self.hw - xa + 0.15) / 6)
                    self.area(xs, xs + (self.hw - xa) / 12, y0 + k * 0.3, y0 + k * 0.3 + 0.12,
                              (30, 30, 30))
                k += 1
        for i, (xa, xb) in enumerate(aisles):                    # aisles
            xc = (xa + xb) / 2
            y = -y_end
            while y < y_end:                                     # dashed centre line
                self.area(xc - 0.05, xc + 0.05, y, min(y + 1.0, y_end), white)
                y += 2.0
            for k, yc in enumerate((-y_end + 6, -8.0, 8.0, y_end - 6)):
                d = 1 if (i + k) % 2 == 0 else -1
                self.arrow(xc - 1.2 if d > 0 else xc + 1.2, yc, d)
            for ys in (-y_end - 0.6, y_end + 0.2):               # stop bars at the aisle ends
                self.area(xa + 0.3, xb - 0.3, ys, ys + 0.4, white)
        for sgn in (-1, 1):                                      # walkway across each end
            yw = sgn * (self.hl - 3.0)
            self.area(-self.hw, self.hw, yw - 0.6, yw + 0.6, green)
            self.area(-self.hw, self.hw, yw - 0.66, yw - 0.6, white)
            self.area(-self.hw, self.hw, yw + 0.6, yw + 0.66, white)
            for xa, xb in aisles:                                # zebra crossings into the aisles
                xs = xa + 0.5
                while xs < xb - 0.5:
                    self.area(xs, xs + 0.35, yw - 0.6 - 2.4 * sgn if sgn > 0 else yw + 0.6,
                              yw - 0.6 if sgn > 0 else yw + 0.6 + 2.4, white)
                    xs += 0.7


def make_floor(path, length, width, lines, zone4, aisles, y_end):
    floor = Floor(length, width, 2000)
    floor.concrete()
    floor.wear(aisles)
    floor.markings(lines, zone4, aisles, y_end)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_png(path, floor.w, floor.h, floor.rows)
    print(f"{path}: {floor.w} x {floor.h} px ({length:g} x {width:g} m)")


def make(path, length, height, seed, doors):
    wall = Wall(length, height, seed)
    wall.concrete()
    wall.base_band()
    spans = wall.columns()
    wall.services()
    wall.panels(spans, doors)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_png(path, wall.w, wall.h, wall.rows)
    return wall.w, wall.h


def write_textures(directory, faces, height):
    """faces: (name, inner length [m], number of roll-up doors); one PNG per face."""
    for i, (name, length, doors) in enumerate(faces):
        w, h = make(directory / f"{name}.png", length, height, 1000 + i, doors)
        print(f"{directory / f'{name}.png'}: {w} x {h} px ({length:g} x {height:g} m)")



# --- roof underside ------------------------------------------------------------------------------

class Roof(Floor):
    """Underside of the roof deck seen from below, same mapping as the floor (u along world Y,
    v along world X): corrugated steel sheet, sheet joints, fasteners on the purlins, stains,
    skylight panels."""
    ppm = 20          # 5 cm/px: 6 m below the camera, at the edge of its field of view
    base = (176, 178, 182)

    def metal(self, purlins_x):
        rng = self.rng
        step = 3.0 * self.ppm
        gw, gh = int(self.w / step) + 2, int(self.h / step) + 2
        grid = [[rng.uniform(-1, 1) for _ in range(gw)] for _ in range(gh)]
        cols = []
        for g in grid:
            line = []
            for x in range(self.w):
                k, f = divmod(x / step, 1)
                k = int(k)
                line.append(g[k] * (1 - f) + g[k + 1] * f)
            cols.append(line)
        rib = (1.10, 1.02, 0.90, 0.98)          # trapezoidal ribs every 0.2 m along Y
        joint = self.ppm                         # sheet joints every 1 m along X
        for r in range(self.h):
            z = self.h - 1 - r
            k, f = divmod(z / step, 1)
            k = int(k)
            s0, s1 = cols[k], cols[k + 1]
            joint_row = z % joint == 0
            row = self.rows[r]
            for x in range(self.w):
                v = rib[x % 4] * (1 + 0.10 * (s0[x] * (1 - f) + s1[x] * f)) + rng.uniform(-0.03, 0.03)
                if joint_row:
                    v *= 0.7
                i = 3 * x
                row[i] = min(255, max(0, int(self.base[0] * v)))
                row[i + 1] = min(255, max(0, int(self.base[1] * v)))
                row[i + 2] = min(255, max(0, int(self.base[2] * v)))
        for xp in purlins_x:                     # fastener rows along the purlins
            y = -self.hl
            while y < self.hl:
                self.area(xp - 0.03, xp + 0.03, y, y + 0.06, (90, 92, 95))
                y += 0.4

    def skylights(self, panels):
        for xa, xb, ya, yb in panels:
            self.area(xa - 0.08, xb + 0.08, ya - 0.08, yb + 0.08, (95, 98, 102))     # frame
            self.area(xa, xb, ya, yb, (232, 236, 240))
            y = ya + 0.5
            while y < yb:                                                        # glazing bars
                self.area(xa, xb, y, y + 0.04, (200, 204, 208))
                y += 0.5


def make_roof(path, length, width, purlins_x, skylights):
    roof = Roof(length, width, 4000)
    roof.metal(purlins_x)
    roof.skylights(skylights)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_png(path, roof.w, roof.h, roof.rows)
    print(f"{path}: {roof.w} x {roof.h} px ({length:g} x {width:g} m)")


# --- cartons -------------------------------------------------------------------------------------

CARTON_CELL = 128         # px per atlas cell (a carton face is stretched over one cell)
CARTON_GRID = 4           # 4 x 4 cells: cells 0-7 = sides of variants 0-7, cells 8-15 = their tops


class Canvas:
    """Pixel canvas, (x, y) from the top left corner."""

    def __init__(self, w, h, color):
        self.w, self.h = w, h
        self.rows = [bytearray(bytes(color) * w) for _ in range(h)]

    def rect(self, x0, y0, x1, y1, color):
        x0, x1 = max(0, int(x0)), min(self.w, int(x1))
        if x1 <= x0:
            return
        fill = bytes(color) * (x1 - x0)
        for y in range(max(0, int(y0)), min(self.h, int(y1))):
            self.rows[y][3 * x0:3 * x1] = fill

    def shade(self, x0, y0, x1, y1, f):
        for y in range(max(0, int(y0)), min(self.h, int(y1))):
            row = self.rows[y]
            for i in range(3 * max(0, int(x0)), 3 * min(self.w, int(x1))):
                row[i] = min(255, int(row[i] * f))


def carton_cell(cv, rng, x0, y0, top, base, white):
    c = CARTON_CELL
    for y in range(c):
        row = cv.rows[y0 + y]
        edge_y = min(y, c - 1 - y)
        for x in range(c):
            edge = min(edge_y, x, c - 1 - x)
            v = 1 + rng.uniform(-0.06, 0.06)
            if edge < 5:                                         # worn, darker edges
                v *= 0.78 + 0.044 * edge
            if not top and y % 3 == 0:                           # faint flutes of the board
                v *= 0.985
            i = 3 * (x0 + x)
            for k in range(3):
                row[i + k] = min(255, max(0, int(base[k] * v)))
    ink = (58, 44, 34) if not white else (40, 60, 120)
    if top:
        # packing tape along the long side, over the flap seam; some cross-taped
        tape = (rng.randint(205, 225), rng.randint(180, 200), rng.randint(125, 150))
        cv.rect(x0, y0 + c // 2 - 1, x0 + c, y0 + c // 2 + 1, (90, 70, 50))      # seam
        cv.rect(x0, y0 + c // 2 - 8, x0 + c, y0 + c // 2 + 8, tape)
        cv.rect(x0, y0 + c // 2 - 6, x0 + c, y0 + c // 2 - 4, tuple(min(255, v + 25) for v in tape))
        if rng.random() < 0.35:
            cv.rect(x0 + c // 2 - 8, y0, x0 + c // 2 + 8, y0 + c, tape)
        if rng.random() < 0.5:                                   # a marker scribble / sticker
            sx, sy = x0 + rng.randint(10, 80), y0 + rng.randint(10, 30)
            cv.rect(sx, sy, sx + rng.randint(15, 35), sy + rng.randint(8, 14), (235, 235, 230))
    else:
        if rng.random() < 0.6:                                   # printed maker block
            bx, by = x0 + rng.randint(10, 40), y0 + rng.randint(60, 90)
            cv.rect(bx, by, bx + rng.randint(40, 70), by + 9, ink)
            for k in range(2):
                cv.rect(bx, by + 14 + 6 * k, bx + rng.randint(25, 60), by + 16 + 6 * k, ink)
        if rng.random() < 0.6:                                   # "this way up" arrows
            ax, ay = x0 + rng.randint(80, 104), y0 + 12
            for a in (0, 11):
                cv.rect(ax + a + 3, ay + 6, ax + a + 5, ay + 18, ink)
                for j in range(5):
                    cv.rect(ax + a + 4 - j, ay + j + 1, ax + a + 5 + j, ay + j + 2, ink)
            cv.rect(ax - 1, ay + 20, ax + 20, ay + 22, ink)
        if rng.random() < 0.4:                                   # hand hole
            hx, hy = x0 + c // 2 - 14, y0 + 26
            cv.rect(hx + 2, hy, hx + 26, hy + 9, (52, 40, 30))
            cv.rect(hx, hy + 2, hx + 28, hy + 7, (52, 40, 30))
        if rng.random() < 0.3:                                   # red "fragile" band
            cv.rect(x0 + 6, y0 + c - 24, x0 + 60, y0 + c - 14, (170, 40, 35))


def make_carton_atlas(path, seed=3000):
    """Atlas of carton faces: cells 0-7 = sides of the 8 variants, cells 8-15 = their tops."""
    rng = random.Random(seed)
    n, c = CARTON_GRID, CARTON_CELL
    cv = Canvas(n * c, n * c, (0, 0, 0))
    # one board colour per variant, shared by its side (cell k) and its top (cell 8 + k);
    # 2 of the 8 variants are white printed board
    variants = []
    for k in range(n * n // 2):
        white = k in (3, 6)
        variants.append(((214, 210, 200) if white else
                         (rng.randint(176, 200), rng.randint(136, 156), rng.randint(88, 106)), white))
    for cell in range(n * n):
        col, row = cell % n, cell // n
        base, white = variants[cell % (n * n // 2)]
        carton_cell(cv, rng, col * c, row * c, row >= n // 2, base, white)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_png(path, cv.w, cv.h, cv.rows)


# --- GS1-style pallet labels ---------------------------------------------------------------------

LABEL_W, LABEL_H, LABEL_COLS = 128, 192, 32     # px per label (A5, ~1.2 mm/px), labels per atlas row
EAN_L = ['0001101', '0011001', '0010011', '0111101', '0100011', '0110001', '0101111', '0111011',
         '0110111', '0001011']
EAN_G = ['0100111', '0110011', '0011011', '0100001', '0011101', '0111001', '0000101', '0010001',
         '0001001', '0010111']
EAN_R = [''.join('1' if b == '0' else '0' for b in code) for code in EAN_L]
EAN_PARITY = ['LLLLLL', 'LLGLGG', 'LLGGLG', 'LLGGGL', 'LGLLGG', 'LGGLLG', 'LGGGLG', 'LGLGGL',
              'LGLGLG', 'LGGLGL']
ITF = ['nnwwn', 'wnnnw', 'nwnnw', 'wwnnn', 'nnwnw', 'wnwnn', 'nwwnn', 'nnnww', 'wnnwn', 'nwnwn']
DIGITS = ['111101101101111', '010110010010111', '111001111100111', '111001111001111',
          '101101111001001', '111100111001111', '111100111101111', '111001010010010',
          '111101111101111', '111101111001111']   # 3 x 5 pixel font


def gs1_check(data):
    """GS1 check digit: weights 3, 1, 3, ... from the rightmost data digit."""
    s = sum(int(d) * (3 if i % 2 == 0 else 1) for i, d in enumerate(reversed(data)))
    return str((10 - s % 10) % 10)


def ean13(code12):
    code = code12 + gs1_check(code12)
    left = ''.join((EAN_L if p == 'L' else EAN_G)[int(d)] for p, d in zip(EAN_PARITY[int(code[0])], code[1:7]))
    right = ''.join(EAN_R[int(d)] for d in code[7:])
    return code, '101' + left + '01010' + right + '101'


def itf14(code13):
    """ITF-14 (interleaved 2 of 5), narrow = 1, wide = 2 modules: '1' bar, '0' space."""
    code = code13 + gs1_check(code13)
    w = {'n': 1, 'w': 2}
    bits = '1010'
    for a, b in zip(code[0::2], code[1::2]):
        for ka, kb in zip(ITF[int(a)], ITF[int(b)]):
            bits += '1' * w[ka] + '0' * w[kb]
    return code, bits + '11' + '0' + '1'


def label_rows(n):
    return (n + LABEL_COLS - 1) // LABEL_COLS


def draw_digits(cv, x, y, text, color):
    for k, d in enumerate(text):
        bits = DIGITS[int(d)]
        for j in range(15):
            if bits[j] == '1':
                cv.rect(x + 4 * k + j % 3, y + j // 3, x + 4 * k + j % 3 + 1, y + j // 3 + 1, color)


def draw_bars(cv, x, y0, y1, bits, color):
    for k, b in enumerate(bits):
        if b == '1':
            cv.rect(x + k, y0, x + k + 1, y1, color)


def make_label_atlas(path, seeds):
    """One GS1-style logistics label per seed: header, text lines and one ITF-14 barcode (case
    code: indicator 1 + a GTIN-12 with the GS1 Italy prefix 80-83), valid check digit and
    human-readable digits."""
    rows = label_rows(len(seeds))
    cv = Canvas(LABEL_COLS * LABEL_W, rows * LABEL_H, (246, 246, 242))
    ink, grey = (22, 22, 22), (120, 120, 120)
    for idx, seed in enumerate(seeds):
        rng = random.Random(seed)
        x0, y0 = (idx % LABEL_COLS) * LABEL_W, (idx // LABEL_COLS) * LABEL_H
        cv.rect(x0, y0, x0 + LABEL_W, y0 + 1, grey)                          # border
        cv.rect(x0, y0 + LABEL_H - 1, x0 + LABEL_W, y0 + LABEL_H, grey)
        cv.rect(x0, y0, x0 + 1, y0 + LABEL_H, grey)
        cv.rect(x0 + LABEL_W - 1, y0, x0 + LABEL_W, y0 + LABEL_H, grey)
        cv.rect(x0 + 6, y0 + 5, x0 + 6 + rng.randint(50, 90), y0 + 11, ink)  # company name
        for k in range(4):                                                   # text lines
            cv.rect(x0 + 6, y0 + 16 + 6 * k, x0 + 6 + rng.randint(30, 110), y0 + 18 + 6 * k, grey)
        for k in range(2):                                                   # more text lines
            cv.rect(x0 + 6, y0 + 46 + 6 * k, x0 + 6 + rng.randint(30, 80), y0 + 48 + 6 * k, grey)
        cv.rect(x0 + 4, y0 + 62, x0 + LABEL_W - 4, y0 + 63, ink)
        gtin12 = '8' + str(rng.randint(0, 3)) + ''.join(str(rng.randint(0, 9)) for _ in range(10))
        case, ibits = itf14('1' + gtin12)
        ix = x0 + (LABEL_W - len(ibits)) // 2
        draw_bars(cv, ix, y0 + 82, y0 + 166, ibits, ink)
        cv.rect(ix - 4, y0 + 77, ix + len(ibits) + 4, y0 + 81, ink)         # bearer bars
        cv.rect(ix - 4, y0 + 167, ix + len(ibits) + 4, y0 + 171, ink)
        draw_digits(cv, x0 + (LABEL_W - 4 * 14) // 2, y0 + 177, case, ink)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_png(path, cv.w, cv.h, cv.rows)

if __name__ == "__main__":
    import gen_warehouse_layout as layout
    write_textures(layout.MODELS / "warehouse_walls" / "materials" / "textures",
                   layout.wall_faces(), layout.WALL_H)
    layout.floor_texture()
    layout.roof_texture()
