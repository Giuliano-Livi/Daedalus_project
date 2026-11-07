#!/usr/bin/env python3
"""Generate the warehouse layout from Draft_WAREHOUSE (L. Vianello, 20/10/26).

For performance each shelf line is ONE static model (gazebo/models/warehouse_line_N):
  - visuals: merged OBJ meshes (posts, boards, pallets, cartons, tape) -> a few draw calls
    instead of thousands; every post, board, pallet plank and box is modelled,
    so cameras / depth sensors see the full detail.
  - collisions: merged boxes (touching posts -> one post, each level -> one board,
    one bounding box per pallet).
Each pallet carries one textured carton (sizes and texture variants differing, misaligned with
gaussian noise) and a GS1-style label with one barcode on the side facing the aisle; collisions stay
one box per pallet + carton.
The roof (gazebo/models/warehouse_roof: deck at the top of the 9 m walls, I-beams, purlins) and the
lighting (gazebo/models/warehouse_lights: LED fixtures + point lights, one entity in the GUI).
The walls are a model too (gazebo/models/warehouse_walls), with their inner faces textured by
scripts/gen_warehouse_textures.py so the cameras find features on them; same for the floor
(gazebo/models/warehouse_floor, with lane markings following the layout).
Then it rewrites the block between the BEGIN/END "draft layout" markers in
gazebo/worlds/warehouse_px4.sdf with the walls, the 3 line models and the zone 4
model (gazebo/models/warehouse_zone_4, floor pallets), and writes
gazebo/worlds/pallet_slots.csv: the top-surface center of every pallet, where a
box model (origin at its bottom center) can be placed.

Draft frame -> world frame:
  draft x (70 m, left -> right)  -> world Y = x - 35
  draft y (30 m, top  -> bottom) -> world X = y - 15
so the building is centered on the world origin and the shelf lines run along Y.

Draft dimensions (m), measured from the outer wall face:
  walls 1.0 thick (not dimensioned, measured from the drawing)
  line 1: single shelf row, 1.3 deep, against the top wall
  aisle 5.0 | line 2: double row, 2.6 | aisle 5.0 | line 3: double row, 2.6
  aisle 7.0 | zone 4: 46 x 4.5 area, floor pallets in a block (5 x 35, 0.1 m gaps),
             each with a carton box of random height (1.0 / 1.6 / 2.0 m total)
  lines 1-3 run from x = 10 to x = 60 (50 m -> 25 shelves of 2 m)

Usage: scripts/gen_warehouse_layout.py
"""
import math
import random
import zlib
from pathlib import Path

import gen_warehouse_textures

ROOT = Path(__file__).resolve().parent.parent
WORLD = ROOT / "gazebo/worlds/warehouse_px4.sdf"
SLOTS = ROOT / "gazebo/worlds/pallet_slots.csv"
MODELS = ROOT / "gazebo/models"
BEGIN = "<!-- BEGIN draft layout"
END = "<!-- END draft layout -->"

LENGTH, WIDTH = 70.0, 30.0      # outer building size (draft x, draft y)
WALL_T, WALL_H = 1.0, 9.0       # wall thickness and height (up to the roof)
SHELF_DEPTH, SHELF_LEN = 1.3, 2.0
LINE_X0, LINE_X1 = 10.0, 60.0   # draft x extent of shelf lines 1-3
PALLET_LEVELS = (0.0, 2.0, 4.0)  # z of the floor and of the shelf boards' top surfaces; () = no pallets

# Shelf geometry (1.3 x 2.0 m bays, boards at 2 and 4 m, posts 0.5 m above the top board)
POST, POST_H = 0.08, 4.5        # corner post section and height
BOARD_T, BOARD_TOPS = 0.04, (2.0, 4.0)

# EUR pallet (EPAL 1)
PALLET_L, PALLET_W, PALLET_H = 1.2, 0.8, 0.144

COLORS = {"posts": "0.1 0.2 0.5", "boards": "0.6 0.45 0.25", "pallets": "0.72 0.58 0.38",
          "cartons": "0.76 0.6 0.4", "tape": "0.55 0.4 0.22"}

# (number, draft y of the line's top edge, number of shelves back to back)
LINES = [
    (1, WALL_T, 1),
    (2, WALL_T + 1.3 + 5.0, 2),
    (3, WALL_T + 1.3 + 5.0 + 2.6 + 5.0, 2),
]

# Zone 4: floor block storage of pallets (draft x 12..58, from 7 m below line 3 to the wall)
ZONE4_X0, ZONE4_X1 = 12.0, 58.0
ZONE4_TOP = LINES[-1][1] + 2.6 + 7.0
ZONE4_GAP = 0.1                 # gap between neighbouring pallets

# Roof: deck underside at the top of the walls, steel I-beams across the building every 6 m,
# purlins along it every 1.5 m, linear LED high-bay fixtures hanging over the aisles and zone 4
ROOF_Z = WALL_H
BEAM_YS = [-30.0 + 6.0 * k for k in range(11)]
BEAM_DEPTH, FLANGE_W, FLANGE_T, WEB_T = 0.6, 0.3, 0.025, 0.012
PURLIN_XS = [-13.5 + 1.5 * k for k in range(19)]
PURLIN_DEPTH, PURLIN_W = 0.12, 0.06
LAMP_Z = 7.5                    # bottom of the fixtures, above the top boxes (6 m)
LAMP_YS = (-31.5, -27, -21, -15, -9, -3, 3, 9, 15, 21, 27, 31.5)   # between the beams
SKYLIGHT_XS, SKYLIGHT_YS = (-6.0, 6.0), (-27, -15, -3, 9, 21)
# Lights: fewer than the fixtures (each light costs rendering on every pixel it reaches): one row per
# fixture row, LIGHT_YS along it. Fitted to the 48 lights of one per fixture (diffuse 0.5, constant
# 0.25, linear 0.05, quadratic 0.008, range 12 m): same mean light on the floor and on the shelf
# faces (within 1%), brightest spots within 3%, darkest points between lights ~9% darker
LIGHT_YS = (-28, -14, 0, 14, 28)
LIGHT_ATTENUATION = {"range": 12, "constant": 0.25, "linear": 0.2, "quadratic": 0.004}
LIGHT_INTENSITY = 4.08
ROOF_COLORS = {"beams": "0.22 0.27 0.38", "purlins": "0.62 0.63 0.65", "fixtures": "0.78 0.79 0.80",
               "cables": "0.08 0.08 0.08"}
LIGHTS_MODEL = "warehouse_lights"   # fixtures + all the lights: one entity in the GUI

# Carton loads on the pallets: one carton per pallet, each a little different (size, texture
# variant), misaligned with gaussian noise (a share of "sloppy" pallets with more), always clear of
# the neighbouring loads; one GS1-style label (one barcode) on the side facing the aisle.
# carton_load also builds layered loads ("2x2", "3x2" cases per layer) if LOAD_MIX asks for them
LOAD_SEED = 7
LOAD_MIX = (("single", 1.0),)
STACK_SIGMA = (0.015, math.radians(1.0))       # stack offset (m), yaw (rad)
SLOPPY = (0.25, 0.04, math.radians(3.5))       # share of sloppy pallets, their offset / yaw sigma
CASE_SIGMA = (0.005, math.radians(0.8))        # each case inside a stack
LABEL_SIZE = (0.148, 0.210)                    # A5 label, width x height (m)

# Carton boxes (pallet footprint 1.2 x 0.8 m). Types by TOTAL height from the floor,
# pallet included: box height = total - PALLET_H. Named carton_box_<cm> in pallet_slots.csv.
BOX_TOTAL_H = (1.0, 1.6, 2.0)
ZONE4_BOX_SEED = 4              # change it for a different random disposition in zone 4

# Shelf pallets (lines 1-3): share of all shelf slots per box type (total height, None = empty
# pallet). Types taller than a level's clear height (1.96 m) can only go on the top level.
SHELF_BOX_MIX = {1.0: 0.40, 1.6: 0.30, 2.0: 0.15, None: 0.15}
SHELF_BOX_SEED = 7              # change it for a different random disposition on the shelves


def to_world(dx, dy):
    return dy - WIDTH / 2, dx - LENGTH / 2


def pallet_parts():
    """EPAL 1 boxes (cx, cy, cz, sx, sy, sz), length along X, origin at the bottom center."""
    t, bh = 0.022, 0.078
    parts = []
    for y, w in ((-0.35, 0.100), (0.0, 0.145), (0.35, 0.100)):            # bottom boards
        parts.append((0.0, y, t / 2, PALLET_L, w, t))
    for x in (-0.5275, 0.0, 0.5275):                                      # 9 blocks
        for y, w in ((-0.35, 0.100), (0.0, 0.145), (0.35, 0.100)):
            parts.append((x, y, t + bh / 2, 0.145, w, bh))
    for x in (-0.5275, 0.0, 0.5275):                                      # stringer boards
        parts.append((x, 0.0, t + bh + t / 2, 0.145, PALLET_W, t))
    for y, w in ((-0.3275, 0.145), (-0.16375, 0.100), (0.0, 0.145),       # top boards
                 (0.16375, 0.100), (0.3275, 0.145)):
        parts.append((0.0, y, 2 * t + bh + t / 2, PALLET_L, w, t))
    return parts


def carton_load(rng, h, lim, face):
    """Cartons of one pallet load of height h in the pallet frame (x along the 1.2 m side, origin at
    the centre of the pallet top): [cx, cy, cz, sx, sy, sz, yaw, side cell, top cell], plus the
    label (carton index, face, horizontal offset, height on the carton). lim = allowed half extents
    (x, y) of the load, so it stays clear of the neighbouring loads."""
    kind = rng.choices([k for k, _ in LOAD_MIX], [w for _, w in LOAD_MIX])[0]
    nx, ny, layer_h = {"single": (1, 1, h), "2x2": (2, 2, 0.4), "3x2": (3, 2, 0.3)}[kind]
    layers = 1 if kind == "single" else max(1, round(h / layer_h))
    hl, cw, cd = h / layers, PALLET_L / nx, PALLET_W / ny
    multi = kind != "single"
    clip = lambda v, m: max(-m, min(m, v))
    cartons = []
    for layer in range(layers):
        for i in range(nx):
            for j in range(ny):
                sx = cw - 0.01 - rng.uniform(0, 0.02 if multi else 0.05)
                sy = cd - 0.01 - rng.uniform(0, 0.02 if multi else 0.05)
                cx = -PALLET_L / 2 + cw * (i + 0.5) + (clip(rng.gauss(0, CASE_SIGMA[0]), 0.01) if multi else 0)
                cy = -PALLET_W / 2 + cd * (j + 0.5) + (clip(rng.gauss(0, CASE_SIGMA[0]), 0.01) if multi else 0)
                yaw = clip(rng.gauss(0, CASE_SIGMA[1]), math.radians(2)) if multi else 0.0
                variant = rng.randrange(8)              # side cell, its top = cell 8 + variant
                cartons.append([cx, cy, layer * hl + hl / 2, sx, sy, hl - 0.002, yaw,
                                variant, 8 + variant])
    # label: bottom-layer carton on the aisle face, the one nearest the middle of that face
    if face == "+x":
        pick = min(range(nx * ny), key=lambda k: (k // ny != nx - 1, abs(k % ny - (ny - 1) / 2)))
    elif face == "-x":
        pick = min(range(nx * ny), key=lambda k: (k // ny != 0, abs(k % ny - (ny - 1) / 2)))
    else:                                                       # "+y"
        pick = min(range(nx * ny), key=lambda k: (k % ny != ny - 1, abs(k // ny - (nx - 1) / 2)))
    sz = cartons[pick][5]
    lo, hi = -sz / 2 + LABEL_SIZE[1] / 2 + 0.01, sz / 2 - LABEL_SIZE[1] / 2 - 0.01
    label = (pick, face, clip(rng.gauss(0, 0.02), 0.05),
             max(lo, min(hi, -sz / 2 + (0.45 if not multi else 0.55 * sz))))
    # whole stack misaligned on the pallet
    s_xy, s_yaw = (SLOPPY[1], SLOPPY[2]) if rng.random() < SLOPPY[0] else STACK_SIGMA
    yaw_s = clip(rng.gauss(0, s_yaw), math.radians(6))
    hx, hy = PALLET_L / 2 + 0.012, PALLET_W / 2 + 0.012
    while True:
        c, sn = math.cos(abs(yaw_s)), math.sin(abs(yaw_s))
        ex, ey = hx * c + hy * sn, hx * sn + hy * c
        if (ex <= lim[0] and ey <= lim[1]) or abs(yaw_s) < 1e-4:
            break
        yaw_s /= 2
    dx = clip(rng.gauss(0, s_xy), max(0.0, lim[0] - ex))
    dy = clip(rng.gauss(0, s_xy), max(0.0, lim[1] - ey))
    c, sn = math.cos(yaw_s), math.sin(yaw_s)
    for k in cartons:
        k[0], k[1] = c * k[0] - sn * k[1] + dx, sn * k[0] + c * k[1] + dy
        k[6] += yaw_s
    return cartons, label


def place_load(cartons, label, wx, wy, z0, base_yaw, label_id):
    """Pallet frame -> world: rotation base_yaw, pallet centre (wx, wy), pallet top at z0."""
    c, sn = math.cos(base_yaw), math.sin(base_yaw)
    out = [(wx + c * cx - sn * cy, wy + sn * cx + c * cy, z0 + cz, sx, sy, sz, base_yaw + yaw, side, top)
           for cx, cy, cz, sx, sy, sz, yaw, side, top in cartons]
    pick, face, oh, zc = label
    return out, (out[pick], face, oh, zc, label_id)


def box_name(total_h):
    return f"carton_box_{round(total_h * 100)}"


def shelf_slot_keys():
    """(line, row, shelf, level) of every pallet on the shelves, in generation order."""
    n = round((LINE_X1 - LINE_X0) / SHELF_LEN)
    return [(num, r + 1, i + 1, k) for num, _, rows in LINES for r in range(rows)
            for i in range(n) for k in range(len(PALLET_LEVELS))]


def assign_shelf_boxes(keys):
    """Random box type (total height or None) per shelf slot, with exact SHELF_BOX_MIX counts."""
    # exact counts: largest remainder rounding so they add up to len(keys)
    raw = {t: f * len(keys) for t, f in SHELF_BOX_MIX.items()}
    counts = {t: int(v) for t, v in raw.items()}
    for t in sorted(raw, key=lambda t: raw[t] - counts[t], reverse=True)[:len(keys) - sum(counts.values())]:
        counts[t] += 1
    clear = BOARD_TOPS[0] - BOARD_T - PALLET_LEVELS[0]     # clear height under a board
    top_level = len(PALLET_LEVELS) - 1
    upper = [k for k in keys if k[3] == top_level]
    rng = random.Random(SHELF_BOX_SEED)
    out, free = {}, list(keys)
    # tall types first, only on the top level, one box per slot: if they don't all fit,
    # place fewer of them and leave the extra slots empty
    counts.setdefault(None, 0)
    for t in sorted((t for t in counts if t is not None and t > clear), reverse=True):
        avail = [k for k in upper if k not in out]
        if counts[t] > len(avail):
            print(f"warning: only {len(avail)} free top-level slots, placing {len(avail)} "
                  f"of {counts[t]} boxes of {t} m (the rest stay empty)")
            counts[None] += counts[t] - len(avail)
            counts[t] = len(avail)
        for k in rng.sample(avail, counts[t]):
            out[k] = t
    free = [k for k in keys if k not in out]
    rng.shuffle(free)
    for t in (t for t in counts if t is None or t <= clear):
        for k in free[:counts[t]]:
            out[k] = t
        free = free[counts[t]:]
    return out, counts


def line_geometry(num, top, rows, boxes, rng):
    """Visual boxes per part, collision boxes, pallet slots, cartons and labels of one line (world
    frame). boxes: (line, row, shelf, level) -> carton total height or None, from assign_shelf_boxes."""
    n = round((LINE_X1 - LINE_X0) / SHELF_LEN)
    x0, _ = to_world(0, top)
    y0, y1 = to_world(LINE_X0, 0)[1], to_world(LINE_X1, 0)[1]
    depth = SHELF_DEPTH * rows
    vis = {"posts": [], "boards": [], "pallets": []}
    col, slots, cartons, labels = [], [], [], []
    pp = pallet_parts()
    for r in range(rows):
        wx = x0 + SHELF_DEPTH * (r + 0.5)
        for i in range(n):
            wy = y0 + SHELF_LEN * (i + 0.5)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    vis["posts"].append((wx + sx * (SHELF_DEPTH - POST) / 2,
                                         wy + sy * (SHELF_LEN - POST) / 2,
                                         POST_H / 2, POST, POST, POST_H))
            for k, z in enumerate(PALLET_LEVELS):
                vis["pallets"] += [(wx + cx, wy + cy, z + cz, a, b, c) for cx, cy, cz, a, b, c in pp]
                total = boxes[(num, r + 1, i + 1, k)]
                if total is None:
                    col.append((wx, wy, z + PALLET_H / 2, PALLET_L, PALLET_W, PALLET_H))
                    slots.append((r + 1, i + 1, k, wx, wy, z + PALLET_H, ""))
                    continue
                # label on the face towards the aisle: the single row against the wall faces +X,
                # back-to-back rows face away from each other
                face = "+x" if rows == 1 or r == rows - 1 else "-x"
                load, label = carton_load(rng, total - PALLET_H, (0.645, 0.48), face)
                load, label = place_load(load, label, wx, wy, z + PALLET_H, 0.0, len(labels))
                cartons += load
                labels.append(label)
                # pallet + box share the footprint: one collision box up to the top of the carton
                col.append((wx, wy, z + total / 2, PALLET_L, PALLET_W, total))
                slots.append((r + 1, i + 1, k, wx, wy, z + PALLET_H, box_name(total)))
    # boards: one continuous board per level (neighbouring and back-to-back boards touch)
    for z in BOARD_TOPS:
        b = (x0 + depth / 2, (y0 + y1) / 2, z - BOARD_T / 2, depth, y1 - y0, BOARD_T)
        vis["boards"].append(b)
        col.append(b)
    # posts: touching posts of neighbouring shelves merged into one
    xs = [(x0 + POST / 2, POST)] + [(x0 + SHELF_DEPTH * k, 2 * POST) for k in range(1, rows)] \
        + [(x0 + depth - POST / 2, POST)]
    ys = [(y0 + POST / 2, POST)] + [(y0 + SHELF_LEN * j, 2 * POST) for j in range(1, n)] \
        + [(y1 - POST / 2, POST)]
    col += [(x, y, POST_H / 2, sx, sy, POST_H) for x, sx in xs for y, sy in ys]
    return vis, col, slots, cartons, labels


def zone4_geometry(load_rng):
    """Floor pallets in zone 4 (0.8 m side across the zone depth, 1.2 m along it),
    each with a carton load of a random height from BOX_TOTAL_H."""
    x0, y0 = to_world(ZONE4_X0, ZONE4_TOP)
    _, y1 = to_world(ZONE4_X1, ZONE4_TOP)
    depth = WIDTH - WALL_T - ZONE4_TOP
    nx = int((depth + ZONE4_GAP) // (PALLET_W + ZONE4_GAP))
    ny = int((y1 - y0 + ZONE4_GAP) // (PALLET_L + ZONE4_GAP))
    # center the block in the zone
    mx = (depth - nx * PALLET_W - (nx - 1) * ZONE4_GAP) / 2
    my = (y1 - y0 - ny * PALLET_L - (ny - 1) * ZONE4_GAP) / 2
    rot = lambda parts: [(cy, cx, cz, b, a, c) for cx, cy, cz, a, b, c in parts]  # 90 deg
    pp = rot(pallet_parts())
    rng = random.Random(ZONE4_BOX_SEED)
    vis, col, slots, cartons, labels = {"pallets": []}, [], [], [], []
    for i in range(nx):
        wx = x0 + mx + PALLET_W / 2 + i * (PALLET_W + ZONE4_GAP)
        for j in range(ny):
            wy = y0 + my + PALLET_L / 2 + j * (PALLET_L + ZONE4_GAP)
            total = rng.choice(BOX_TOTAL_H)
            vis["pallets"] += [(wx + cx, wy + cy, cz, a, b, c) for cx, cy, cz, a, b, c in pp]
            # pallet turned 90 deg: its local +y face looks at -X, the aisle west of the zone;
            # loads 0.1 m apart -> 0.045 m of room on each side
            load, label = carton_load(load_rng, total - PALLET_H, (0.645, 0.445), "+y")
            load, label = place_load(load, label, wx, wy, PALLET_H, math.pi / 2, len(labels))
            cartons += load
            labels.append(label)
            # pallet + box share the footprint: one collision box up to the top of the carton
            col.append((wx, wy, total / 2, PALLET_W, PALLET_L, total))
            slots.append((i + 1, j + 1, 0, wx, wy, PALLET_H, box_name(total)))
    return vis, col, slots, cartons, labels


def write_obj(path, boxes):
    """All boxes in one OBJ mesh, 4 vertices per face so the faces are shaded flat."""
    faces = [((0, 0, 1), [(-1, -1), (1, -1), (1, 1), (-1, 1)]),
             ((0, 0, -1), [(-1, 1), (1, 1), (1, -1), (-1, -1)]),
             ((1, 0, 0), [(-1, -1), (1, -1), (1, 1), (-1, 1)]),
             ((-1, 0, 0), [(1, -1), (-1, -1), (-1, 1), (1, 1)]),
             ((0, 1, 0), [(1, -1), (-1, -1), (-1, 1), (1, 1)]),
             ((0, -1, 0), [(-1, -1), (1, -1), (1, 1), (-1, 1)])]
    lines = [f"vn {n[0]} {n[1]} {n[2]}" for n, _ in faces]
    nv = 0
    for cx, cy, cz, sx, sy, sz in boxes:
        h = (sx / 2, sy / 2, sz / 2)
        c = (cx, cy, cz)
        for fi, (n, quad) in enumerate(faces):
            ax = [i for i in range(3) if n[i] == 0]      # the two in-plane axes
            na = [i for i in range(3) if n[i] != 0][0]   # the normal axis
            for u, v in quad:
                p = [0.0, 0.0, 0.0]
                p[na] = c[na] + n[na] * h[na]
                p[ax[0]] = c[ax[0]] + u * h[ax[0]]
                p[ax[1]] = c[ax[1]] + v * h[ax[1]]
                lines.append(f"v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
            a, b, cc, d = nv + 1, nv + 2, nv + 3, nv + 4
            lines.append(f"f {a}//{fi + 1} {b}//{fi + 1} {cc}//{fi + 1}")
            lines.append(f"f {a}//{fi + 1} {cc}//{fi + 1} {d}//{fi + 1}")
            nv += 4
    path.write_text("\n".join(lines) + "\n")


# faces of a box: local corner signs bottom-left, bottom-right, top-right, top-left seen from
# outside (sides: bottom = -z), and the outward normal
BOX_FACES = {
    "+x": ([(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)], (1, 0, 0)),
    "-x": ([(-1, 1, -1), (-1, -1, -1), (-1, -1, 1), (-1, 1, 1)], (-1, 0, 0)),
    "+y": ([(1, 1, -1), (-1, 1, -1), (-1, 1, 1), (1, 1, 1)], (0, 1, 0)),
    "-y": ([(-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)], (0, -1, 0)),
    "+z": ([(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)], (0, 0, 1)),
    "-z": ([(-1, 1, -1), (1, 1, -1), (1, -1, -1), (-1, -1, -1)], (0, 0, -1)),
}


def cell_uv(index, cols, rows, size_px):
    """UV corners (bottom-left, bottom-right, top-right, top-left) of atlas cell `index`
    (row-major from the top left), inset by half a pixel."""
    col, row = index % cols, index // cols
    eu, ev = 0.5 / (cols * size_px[0]), 0.5 / (rows * size_px[1])
    u0, u1 = col / cols + eu, (col + 1) / cols - eu
    v0, v1 = 1 - (row + 1) / rows + ev, 1 - row / rows - ev
    return [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]


class ObjWriter:
    def __init__(self):
        self.lines, self.nv, self.nt, self.nn = [], 0, 0, 0

    def quad(self, pts, uvs, normal):
        self.lines += [f"v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}" for p in pts]
        self.lines += [f"vt {u:.5f} {v:.5f}" for u, v in uvs]
        self.lines.append(f"vn {normal[0]:.4f} {normal[1]:.4f} {normal[2]:.4f}")
        v, t, n = self.nv, self.nt, self.nn + 1
        self.lines.append(f"f {v + 1}/{t + 1}/{n} {v + 2}/{t + 2}/{n} {v + 3}/{t + 3}/{n}")
        self.lines.append(f"f {v + 1}/{t + 1}/{n} {v + 3}/{t + 3}/{n} {v + 4}/{t + 4}/{n}")
        self.nv, self.nt, self.nn = v + 4, t + 4, n

    def save(self, path):
        path.write_text("\n".join(self.lines) + "\n")


def write_cartons(d, name, cartons, labels):
    """Textured carton meshes + one label quad per pallet, their atlases, and the visuals."""
    g = gen_warehouse_textures
    tex = d / "materials" / "textures"
    # file names unique per model: gz-rendering caches textures by file name, so a second
    # "labels.png" (atlas of another size) would silently reuse the first model's one
    for stale in ("cartons.png", "labels.png"):
        if (tex / stale).exists():
            (tex / stale).unlink()
    g.make_carton_atlas(tex / f"cartons_{name}.png")
    g.make_label_atlas(tex / f"labels_{name}.png", [zlib.crc32(f"{name}-{k}".encode()) for k in range(len(labels))])
    cc, lr = g.CARTON_GRID, g.label_rows(len(labels))
    box_obj, label_obj = ObjWriter(), ObjWriter()

    def pose(c):
        cx, cy, cz, sx, sy, sz, yaw = c[:7]
        co, si = math.cos(yaw), math.sin(yaw)
        to_w = lambda x, y, z: (cx + co * x - si * y, cy + si * x + co * y, cz + z)
        rot = lambda n: (co * n[0] - si * n[1], si * n[0] + co * n[1], n[2])
        return to_w, rot, (sx / 2, sy / 2, sz / 2)

    for c in cartons:
        to_w, rot, h = pose(c)
        for face, (signs, normal) in BOX_FACES.items():
            cell = c[8] if face == "+z" else c[7]
            box_obj.quad([to_w(a * h[0], b * h[1], e * h[2]) for a, b, e in signs],
                         cell_uv(cell, cc, cc, (g.CARTON_CELL, g.CARTON_CELL)), rot(normal))
    lw, lh = LABEL_SIZE[0] / 2, LABEL_SIZE[1] / 2
    for c, face, oh, zc, k in labels:
        to_w, rot, h = pose(c)
        e = 0.002
        if face == "+x":
            pts = [(h[0] + e, oh - lw, zc - lh), (h[0] + e, oh + lw, zc - lh),
                   (h[0] + e, oh + lw, zc + lh), (h[0] + e, oh - lw, zc + lh)]
        elif face == "-x":
            pts = [(-h[0] - e, oh + lw, zc - lh), (-h[0] - e, oh - lw, zc - lh),
                   (-h[0] - e, oh - lw, zc + lh), (-h[0] - e, oh + lw, zc + lh)]
        else:
            pts = [(oh + lw, h[1] + e, zc - lh), (oh - lw, h[1] + e, zc - lh),
                   (oh - lw, h[1] + e, zc + lh), (oh + lw, h[1] + e, zc + lh)]
        label_obj.quad([to_w(*p) for p in pts], cell_uv(k, g.LABEL_COLS, lr, (g.LABEL_W, g.LABEL_H)),
                       rot(BOX_FACES[face][1]))
    box_obj.save(d / "meshes" / "cartons.obj")
    label_obj.save(d / "meshes" / "labels.obj")
    stale = d / "meshes" / "tape.obj"           # tape is drawn in the carton texture now
    if stale.exists():
        stale.unlink()
    visual = lambda part, roughness: (
        f'      <visual name="{part}">\n'
        f'        <geometry><mesh><uri>model://{name}/meshes/{part}.obj</uri></mesh></geometry>\n'
        f'        <material>\n'
        f'          <ambient>1 1 1 1</ambient>\n'
        f'          <diffuse>1 1 1 1</diffuse>\n'
        f'          <specular>0.05 0.05 0.05 1</specular>\n'
        f'          <pbr><metal><albedo_map>model://{name}/materials/textures/{part}_{name}.png</albedo_map>'
        f'<roughness>{roughness}</roughness><metalness>0.0</metalness></metal></pbr>\n'
        f'        </material>\n'
        f'      </visual>\n')
    return visual("cartons", 0.95) + visual("labels", 0.6)


def box_sdf(tag, b, pad):
    x, y, z, sx, sy, sz = b
    return (f'{pad}<collision name="{tag}">\n'
            f'{pad}  <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n'
            f'{pad}  <geometry><box><size>{sx:g} {sy:g} {sz:g}</size></box></geometry>\n'
            f'{pad}</collision>\n')


def write_model(name, title, vis, col, cartons=(), labels=()):
    d = MODELS / name
    (d / "meshes").mkdir(parents=True, exist_ok=True)
    visuals = write_cartons(d, name, cartons, labels) if cartons else ""
    for part, boxes in vis.items():
        write_obj(d / "meshes" / f"{part}.obj", boxes)
        c = COLORS[part]
        visuals += (f'      <visual name="{part}">\n'
                    f'        <geometry><mesh><uri>model://{name}/meshes/{part}.obj</uri></mesh></geometry>\n'
                    f'        <material>\n'
                    f'          <ambient>{c} 1</ambient>\n'
                    f'          <diffuse>{c} 1</diffuse>\n'
                    f'          <specular>0.1 0.1 0.1 1</specular>\n'
                    f'        </material>\n'
                    f'      </visual>\n')
    collisions = "".join(box_sdf(f"c{i}", b, "      ") for i, b in enumerate(col))
    (d / "model.sdf").write_text(
        f'<?xml version="1.0" ?>\n'
        f'<!-- GENERATED by scripts/gen_warehouse_layout.py, do not edit by hand.\n'
        f'     {title}: geometry merged into one static model.\n'
        f'     Origin = world origin (include it with pose 0 0 0). -->\n'
        f'<sdf version="1.9">\n'
        f'  <model name="{name}">\n'
        f'    <static>true</static>\n'
        f'    <link name="link">\n{visuals}{collisions}'
        f'    </link>\n'
        f'  </model>\n'
        f'</sdf>\n')
    (d / "model.config").write_text(
        f'<?xml version="1.0" ?>\n'
        f'<model>\n'
        f'    <name>{name}</name>\n'
        f'    <version>1.0</version>\n'
        f'    <sdf version="1.9">model.sdf</sdf>\n'
        f'    <author>\n'
        f'        <name>Giuliano Livi</name>\n'
        f'        <email></email>\n'
        f'    </author>\n'
        f'    <description>{title} of the warehouse draft, '
        f'generated by scripts/gen_warehouse_layout.py.</description>\n'
        f'</model>\n')
    return name


def wall_faces():
    """Inner faces textured by gen_warehouse_textures: (name, inner length [m], roll-up doors)."""
    return [("north", WIDTH - 2 * WALL_T, 1), ("south", WIDTH - 2 * WALL_T, 1),
            ("east", LENGTH - 2 * WALL_T, 2), ("west", LENGTH - 2 * WALL_T, 2)]


def write_quad(path, corners, uvs=((0, 0), (1, 0), (1, 1), (0, 1))):
    """One textured quad: corners counter-clockwise seen from the inside, starting bottom left,
    so u runs left -> right and v bottom -> top as seen from inside the building."""
    a, b, c = corners[0], corners[1], corners[2]
    e1 = [b[i] - a[i] for i in range(3)]
    e2 = [c[i] - b[i] for i in range(3)]
    n = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]]
    norm = sum(v * v for v in n) ** 0.5
    lines = [f"v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}" for p in corners]
    lines += [f"vt {u:g} {v:g}" for u, v in uvs]
    lines.append(f"vn {n[0] / norm:g} {n[1] / norm:g} {n[2] / norm:g}")
    lines += ["f 1/1/1 2/2/1 3/3/1", "f 1/1/1 3/3/1 4/4/1"]
    path.write_text("\n".join(lines) + "\n")


def walls():
    """Walls model (gazebo/models/warehouse_walls): box collisions, plain outer boxes and, 1 cm in
    front of each inner face, a quad textured by gen_warehouse_textures (features for visual odometry)."""
    hx, hy, t, h = WIDTH / 2, LENGTH / 2, WALL_T, WALL_H
    name = "warehouse_walls"
    d = MODELS / name
    (d / "meshes").mkdir(parents=True, exist_ok=True)
    # name, center x, center y, size x, size y (world frame)
    segs = [
        ("north", 0.0, hy - t / 2, WIDTH, t),
        ("south", 0.0, -hy + t / 2, WIDTH, t),
        ("east", hx - t / 2, 0.0, t, LENGTH - 2 * t),
        ("west", -hx + t / 2, 0.0, t, LENGTH - 2 * t),
    ]
    # inner faces, 1 cm inside the building: corners CCW seen from inside, bottom left first
    ix, iy, eps = hx - t - 0.01, hy - t - 0.01, 0.01
    fx, fy = hx - t, hy - t
    quads = {
        "north": [(-fx, iy, 0), (fx, iy, 0), (fx, iy, h), (-fx, iy, h)],
        "south": [(fx, -iy, 0), (-fx, -iy, 0), (-fx, -iy, h), (fx, -iy, h)],
        "east": [(ix, fy, 0), (ix, -fy, 0), (ix, -fy, h), (ix, fy, h)],
        "west": [(-ix, -fy, 0), (-ix, fy, 0), (-ix, fy, h), (-ix, -fy, h)],
    }
    body = ""
    for n, x, y, sx, sy in segs:
        geom = f"<geometry><box><size>{sx:g} {sy:g} {h:g}</size></box></geometry>"
        write_quad(d / "meshes" / f"{n}.obj", quads[n])
        body += (
            f'      <collision name="{n}_collision">\n'
            f'        <pose>{x:g} {y:g} {h / 2:g} 0 0 0</pose>\n'
            f'        {geom}\n'
            f'      </collision>\n'
            f'      <visual name="{n}_visual">\n'
            f'        <pose>{x:g} {y:g} {h / 2:g} 0 0 0</pose>\n'
            f'        {geom}\n'
            f'        <material>\n'
            f'          <ambient>0.75 0.75 0.72 1</ambient>\n'
            f'          <diffuse>0.75 0.75 0.72 1</diffuse>\n'
            f'        </material>\n'
            f'      </visual>\n'
            f'      <visual name="{n}_inner">\n'
            f'        <geometry><mesh><uri>model://{name}/meshes/{n}.obj</uri></mesh></geometry>\n'
            f'        <material>\n'
            f'          <ambient>1 1 1 1</ambient>\n'
            f'          <diffuse>1 1 1 1</diffuse>\n'
            f'          <specular>0.05 0.05 0.05 1</specular>\n'
            f'          <pbr>\n'
            f'            <metal>\n'
            f'              <albedo_map>model://{name}/materials/textures/{n}.png</albedo_map>\n'
            f'              <roughness>0.9</roughness>\n'
            f'              <metalness>0.0</metalness>\n'
            f'            </metal>\n'
            f'          </pbr>\n'
            f'        </material>\n'
            f'      </visual>\n')
    (d / "model.sdf").write_text(
        f'<?xml version="1.0" ?>\n'
        f'<!-- GENERATED by scripts/gen_warehouse_layout.py, do not edit by hand.\n'
        f'     Walls: {LENGTH:g} x {WIDTH:g} m outer, {WALL_T:g} m thick, {WALL_H:g} m high; inner faces\n'
        f'     textured by scripts/gen_warehouse_textures.py. Origin = world origin. -->\n'
        f'<sdf version="1.9">\n'
        f'  <model name="{name}">\n'
        f'    <static>true</static>\n'
        f'    <link name="link">\n{body}'
        f'    </link>\n'
        f'  </model>\n'
        f'</sdf>\n')
    (d / "model.config").write_text(
        f'<?xml version="1.0" ?>\n'
        f'<model>\n'
        f'    <name>{name}</name>\n'
        f'    <version>1.0</version>\n'
        f'    <sdf version="1.9">model.sdf</sdf>\n'
        f'    <author>\n'
        f'        <name>Giuliano Livi</name>\n'
        f'        <email></email>\n'
        f'    </author>\n'
        f'    <description>Walls of the warehouse draft with textured inner faces, '
        f'generated by scripts/gen_warehouse_layout.py.</description>\n'
        f'</model>\n')
    gen_warehouse_textures.write_textures(d / "materials" / "textures", wall_faces(), h)
    return (f'    <!-- Walls: {LENGTH:g} x {WIDTH:g} m outer, {WALL_T:g} m thick, {WALL_H:g} m high, '
            f'textured inner faces -->\n'
            + include(name, name))


def floor_texture():
    """Floor texture, with the markings at the layout's shelf lines, zone 4 and aisles."""
    hw = WIDTH / 2 - WALL_T
    lines = []
    for num, top, rows in LINES:
        xa = to_world(0, top)[0]
        lines.append((xa, xa + SHELF_DEPTH * rows))
    zx = to_world(0, ZONE4_TOP)[0]
    zone4 = (zx, hw, to_world(ZONE4_X0, 0)[1], to_world(ZONE4_X1, 0)[1])
    edges = sorted(lines) + [(zx, hw)]
    aisles = [(edges[i][1] + 0.15, edges[i + 1][0] - 0.15) for i in range(len(edges) - 1)]
    y_end = to_world(LINE_X1, 0)[1]
    gen_warehouse_textures.make_floor(MODELS / "warehouse_floor" / "materials" / "textures" / "floor.png",
                                      LENGTH - 2 * WALL_T, WIDTH - 2 * WALL_T, lines, zone4, aisles, y_end)


def floor():
    """Floor model (gazebo/models/warehouse_floor): one quad over the inner area, 2 mm above the
    ground plane (which keeps the collision), textured by gen_warehouse_textures."""
    name = "warehouse_floor"
    d = MODELS / name
    (d / "meshes").mkdir(parents=True, exist_ok=True)
    hx, hy, z = WIDTH / 2 - WALL_T, LENGTH / 2 - WALL_T, 0.002
    # u along world Y, v along world X; counter-clockwise seen from above (normal +z)
    write_quad(d / "meshes" / "floor.obj", [(-hx, -hy, z), (hx, -hy, z), (hx, hy, z), (-hx, hy, z)],
               uvs=((0, 0), (0, 1), (1, 1), (1, 0)))
    (d / "model.sdf").write_text(
        f'<?xml version="1.0" ?>\n'
        f'<!-- GENERATED by scripts/gen_warehouse_layout.py, do not edit by hand.\n'
        f'     Floor: visual only (the world ground_plane is the collision), textured by\n'
        f'     scripts/gen_warehouse_textures.py. Origin = world origin. -->\n'
        f'<sdf version="1.9">\n'
        f'  <model name="{name}">\n'
        f'    <static>true</static>\n'
        f'    <link name="link">\n'
        f'      <visual name="floor">\n'
        f'        <geometry><mesh><uri>model://{name}/meshes/floor.obj</uri></mesh></geometry>\n'
        f'        <material>\n'
        f'          <ambient>1 1 1 1</ambient>\n'
        f'          <diffuse>1 1 1 1</diffuse>\n'
        f'          <specular>0.15 0.15 0.15 1</specular>\n'
        f'          <pbr>\n'
        f'            <metal>\n'
        f'              <albedo_map>model://{name}/materials/textures/floor.png</albedo_map>\n'
        f'              <roughness>0.6</roughness>\n'
        f'              <metalness>0.0</metalness>\n'
        f'            </metal>\n'
        f'          </pbr>\n'
        f'        </material>\n'
        f'      </visual>\n'
        f'    </link>\n'
        f'  </model>\n'
        f'</sdf>\n')
    (d / "model.config").write_text(
        f'<?xml version="1.0" ?>\n'
        f'<model>\n'
        f'    <name>{name}</name>\n'
        f'    <version>1.0</version>\n'
        f'    <sdf version="1.9">model.sdf</sdf>\n'
        f'    <author>\n'
        f'        <name>Giuliano Livi</name>\n'
        f'        <email></email>\n'
        f'    </author>\n'
        f'    <description>Floor of the warehouse draft with concrete slabs and lane markings, '
        f'generated by scripts/gen_warehouse_layout.py.</description>\n'
        f'</model>\n')
    floor_texture()
    return ("    <!-- Floor: textured concrete + lane markings (visual only, collision = ground_plane) -->\n"
            + include(name, name))


def lamp_positions():
    """(x, y) of the fixtures: rows over the aisle centres and over zone 4, LAMP_YS along them."""
    hw = WIDTH / 2 - WALL_T
    lines = sorted((to_world(0, top)[0], to_world(0, top)[0] + SHELF_DEPTH * rows) for _, top, rows in LINES)
    zx = to_world(0, ZONE4_TOP)[0]
    edges = lines + [(zx, hw)]
    xs = [(edges[i][1] + edges[i + 1][0]) / 2 for i in range(len(edges) - 1)] + [(zx + hw) / 2]
    return [(x, y) for x in xs for y in LAMP_YS]


def skylights():
    return [(x - 0.75, x + 0.75, y - 1.5, y + 1.5) for x in SKYLIGHT_XS for y in SKYLIGHT_YS]


def roof_texture():
    gen_warehouse_textures.make_roof(MODELS / "warehouse_roof" / "materials" / "textures" / "roof.png",
                                     LENGTH - 2 * WALL_T, WIDTH - 2 * WALL_T, PURLIN_XS, skylights())


def roof():
    """Roof model (gazebo/models/warehouse_roof): deck, steel I-beams across the building, purlins
    along it. The deck is one quad facing DOWN (back faces are culled): a solid roof from inside,
    while the GUI camera above the building still sees in. A thin box at the top of the walls is the
    collision. The fixtures and the lights are their own model (lights())."""
    name = "warehouse_roof"
    d = MODELS / name
    (d / "meshes").mkdir(parents=True, exist_ok=True)
    hx, hy, z = WIDTH / 2 - WALL_T, LENGTH / 2 - WALL_T, ROOF_Z
    write_quad(d / "meshes" / "deck.obj", [(-hx, -hy, z), (-hx, hy, z), (hx, hy, z), (hx, -hy, z)],
               uvs=((0, 0), (1, 0), (1, 1), (0, 1)))
    beams = []
    for y in BEAM_YS:                                   # I-beams: top flange, web, bottom flange
        beams.append((0.0, y, z - FLANGE_T / 2, 2 * hx, FLANGE_W, FLANGE_T))
        beams.append((0.0, y, z - BEAM_DEPTH / 2, 2 * hx, WEB_T, BEAM_DEPTH - 2 * FLANGE_T))
        beams.append((0.0, y, z - BEAM_DEPTH + FLANGE_T / 2, 2 * hx, FLANGE_W, FLANGE_T))
    purlins = [(x, 0.0, z - PURLIN_DEPTH / 2, PURLIN_W, 2 * hy, PURLIN_DEPTH) for x in PURLIN_XS]
    parts = {"beams": beams, "purlins": purlins}
    for part, boxes in parts.items():
        write_obj(d / "meshes" / f"{part}.obj", boxes)
    for stale in ("fixtures", "lenses", "cables"):    # moved to the lights model
        if (d / "meshes" / f"{stale}.obj").exists():
            (d / "meshes" / f"{stale}.obj").unlink()
    roof_texture()
    mesh = lambda part: f'<geometry><mesh><uri>model://{name}/meshes/{part}.obj</uri></mesh></geometry>'
    visuals = (f'      <visual name="deck">\n        {mesh("deck")}\n'
               f'        <material>\n'
               f'          <ambient>1 1 1 1</ambient>\n          <diffuse>1 1 1 1</diffuse>\n'
               f'          <specular>0.3 0.3 0.3 1</specular>\n'
               f'          <pbr><metal><albedo_map>model://{name}/materials/textures/roof.png</albedo_map>'
               f'<roughness>0.5</roughness><metalness>0.3</metalness></metal></pbr>\n'
               f'        </material>\n      </visual>\n')
    for part in parts:
        c = ROOF_COLORS[part]
        visuals += (f'      <visual name="{part}">\n        {mesh(part)}\n'
                    f'        <material>\n          <ambient>{c} 1</ambient>\n          <diffuse>{c} 1</diffuse>\n'
                    f'          <specular>0.3 0.3 0.3 1</specular>\n        </material>\n      </visual>\n')
    collision = (f'      <collision name="deck_collision">\n'
                 f'        <pose>0 0 {z + 0.05:g} 0 0 0</pose>\n'
                 f'        <geometry><box><size>{WIDTH:g} {LENGTH:g} 0.1</size></box></geometry>\n'
                 f'      </collision>\n')
    (d / "model.sdf").write_text(
        f'<?xml version="1.0" ?>\n'
        f'<!-- GENERATED by scripts/gen_warehouse_layout.py, do not edit by hand.\n'
        f'     Roof: deck underside at {z:g} m (one quad facing down, textured by\n'
        f'     scripts/gen_warehouse_textures.py), I-beams, purlins; the fixtures and the lights\n'
        f'     are the {LIGHTS_MODEL} model. Origin = world origin. -->\n'
        f'<sdf version="1.9">\n  <model name="{name}">\n    <static>true</static>\n'
        f'    <link name="link">\n{visuals}{collision}    </link>\n  </model>\n</sdf>\n')
    (d / "model.config").write_text(
        f'<?xml version="1.0" ?>\n<model>\n    <name>{name}</name>\n    <version>1.0</version>\n'
        f'    <sdf version="1.9">model.sdf</sdf>\n    <author>\n        <name>Giuliano Livi</name>\n'
        f'        <email></email>\n    </author>\n'
        f'    <description>Roof of the warehouse draft with beams and purlins, '
        f'generated by scripts/gen_warehouse_layout.py.</description>\n</model>\n')
    return f"    <!-- Roof at {z:g} m: deck, beams, purlins -->\n" + include(name, name)


def lights():
    """Lighting model (gazebo/models/warehouse_lights): the linear LED fixtures hanging on cables
    with an emissive lens, and the point lights (fewer than the fixtures, LIGHT_YS), all in one
    static link, so the GUI shows ONE entity for the whole lighting (shadows off, as in the world)."""
    name = LIGHTS_MODEL
    d = MODELS / name
    (d / "meshes").mkdir(parents=True, exist_ok=True)
    lamps = lamp_positions()
    fixtures, lenses, cables = [], [], []
    top = ROOF_Z - PURLIN_DEPTH
    for x, y in lamps:
        fixtures.append((x, y, LAMP_Z + 0.04, 0.16, 1.2, 0.08))
        lenses.append((x, y, LAMP_Z - 0.003, 0.12, 1.14, 0.006))
        cables += [(x, y + s * 0.5, (LAMP_Z + 0.08 + top) / 2, 0.008, 0.008, top - LAMP_Z - 0.08) for s in (-1, 1)]
    for part, boxes in (("fixtures", fixtures), ("lenses", lenses), ("cables", cables)):
        write_obj(d / "meshes" / f"{part}.obj", boxes)
    mesh = lambda part: f'<geometry><mesh><uri>model://{name}/meshes/{part}.obj</uri></mesh></geometry>'
    visuals = ""
    for part in ("fixtures", "cables"):
        c = ROOF_COLORS[part]
        visuals += (f'      <visual name="{part}">\n        {mesh(part)}\n'
                    f'        <material>\n          <ambient>{c} 1</ambient>\n          <diffuse>{c} 1</diffuse>\n'
                    f'          <specular>0.3 0.3 0.3 1</specular>\n        </material>\n      </visual>\n')
    visuals += (f'      <visual name="lenses">\n        {mesh("lenses")}\n'
                f'        <material>\n          <ambient>1 1 0.96 1</ambient>\n          <diffuse>1 1 0.96 1</diffuse>\n'
                f'          <emissive>1 0.98 0.92 1</emissive>\n        </material>\n      </visual>\n')
    att = "".join(f"<{k}>{v:g}</{k}>" for k, v in LIGHT_ATTENUATION.items())
    light_xy = [(x, y) for x in sorted({x for x, _ in lamps}) for y in LIGHT_YS]
    point_lights = "".join(
        f'      <light name="lamp_{k:02d}" type="point">\n'
        f'        <pose>{x:.2f} {y:.2f} {LAMP_Z - 0.05:g} 0 0 0</pose>\n'
        f'        <cast_shadows>false</cast_shadows>\n'
        f'        <intensity>{LIGHT_INTENSITY:g}</intensity>\n'
        f'        <diffuse>0.50 0.48 0.44 1</diffuse>\n'
        f'        <specular>0.10 0.10 0.10 1</specular>\n'
        f'        <attenuation>{att}</attenuation>\n'
        f'      </light>\n' for k, (x, y) in enumerate(light_xy))
    (d / "model.sdf").write_text(
        f'<?xml version="1.0" ?>\n'
        f'<!-- GENERATED by scripts/gen_warehouse_layout.py, do not edit by hand.\n'
        f'     Lighting: {len(lamps)} LED fixtures + {len(light_xy)} point lights (shadows off),\n'
        f'     one static link so the GUI shows one entity. Origin = world origin. -->\n'
        f'<sdf version="1.9">\n  <model name="{name}">\n    <static>true</static>\n'
        f'    <link name="link">\n{visuals}{point_lights}    </link>\n  </model>\n</sdf>\n')
    (d / "model.config").write_text(
        f'<?xml version="1.0" ?>\n<model>\n    <name>{name}</name>\n    <version>1.0</version>\n'
        f'    <sdf version="1.9">model.sdf</sdf>\n    <author>\n        <name>Giuliano Livi</name>\n'
        f'        <email></email>\n    </author>\n'
        f'    <description>LED fixtures and lights of the warehouse draft, '
        f'generated by scripts/gen_warehouse_layout.py.</description>\n</model>\n')
    return (f"    <!-- Lighting: {len(lamps)} LED fixtures + {len(light_xy)} point lights, one model -->\n"
            + include(name, name))


def include(name, model):
    return (f'    <model name="{name}">\n'
            f'      <include>\n'
            f'        <uri>model://{model}</uri>\n'
            f'      </include>\n'
            f'      <pose>0 0 0 0 0 0</pose>\n'
            f'    </model>\n')


def main():
    # zone 4 rows in the CSV: line=4, row = position across the zone, shelf = position along it
    # box: carton type already on the pallet (empty = free slot)
    incs, rows = [], ["line,row,shelf,level,x,y,z_top,box"]
    shelf_boxes, counts = assign_shelf_boxes(shelf_slot_keys())
    load_rng = random.Random(LOAD_SEED)
    print("shelf slots:", {(box_name(t) if t else "empty"): c for t, c in counts.items()})
    for num, top, nrows in LINES:
        vis, col, slots, cartons, labels = line_geometry(num, top, nrows, shelf_boxes, load_rng)
        model = write_model(f"warehouse_line_{num}", f"Shelf line {num} (shelves + pallets + boxes)", vis, col,
                            cartons, labels)
        incs.append(include(f"shelf_line_{num}", model))
        rows += [f"{num},{r},{s},{lv},{x:.3f},{y:.3f},{z:.3f},{bx}" for r, s, lv, x, y, z, bx in slots]
        print(f"{model}: {sum(len(b) for b in vis.values())} visual boxes in {len(vis)} meshes, "
              f"{len(cartons)} cartons, {len(labels)} labels, {len(col)} collision boxes, {len(slots)} pallets")
    vis, col, slots, cartons, labels = zone4_geometry(load_rng)
    model = write_model("warehouse_zone_4", "Zone 4 (floor pallets + carton boxes)", vis, col, cartons, labels)
    incs.append(include("pallet_zone_4", model))
    rows += [f"4,{r},{s},{lv},{x:.3f},{y:.3f},{z:.3f},{bx}" for r, s, lv, x, y, z, bx in slots]
    counts = {box_name(t): sum(1 for sl in slots if sl[6] == box_name(t)) for t in BOX_TOTAL_H}
    print(f"{model}: {len(col)} floor pallets with boxes {counts}, {len(cartons)} cartons")
    SLOTS.write_text("\n".join(rows) + "\n")

    s = WORLD.read_text()
    a = s.index(BEGIN)
    a = s.index("\n", a) + 1
    b = s.rindex("\n", 0, s.index(END)) + 1
    block = (walls() + "\n" + floor() + "\n" + roof() + "\n" + lights() + "\n"
             + "    <!-- Shelf lines 1-3 (shelves + pallets merged per line) and zone 4 (pallets + boxes) -->\n"
             + "".join(incs))
    WORLD.write_text(s[:a] + block + s[b:])
    print(f"Wrote walls + {len(incs)} merged models to {WORLD}")
    print(f"Wrote {len(rows) - 1} pallet slots to {SLOTS}")


if __name__ == "__main__":
    main()
