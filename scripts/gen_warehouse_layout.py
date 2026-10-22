#!/usr/bin/env python3
"""Generate the warehouse layout from Draft_WAREHOUSE (L. Vianello, 20/10/26).

For performance each shelf line is ONE static model (gazebo/models/warehouse_line_N):
  - visuals: merged OBJ meshes (posts, boards, pallets, cartons, tape) -> a few draw calls
    instead of thousands; every post, board, pallet plank and box is modelled,
    so cameras / depth sensors see the full detail.
  - collisions: merged boxes (touching posts -> one post, each level -> one board,
    one bounding box per pallet).
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
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WORLD = ROOT / "gazebo/worlds/warehouse_px4.sdf"
SLOTS = ROOT / "gazebo/worlds/pallet_slots.csv"
MODELS = ROOT / "gazebo/models"
BEGIN = "<!-- BEGIN draft layout"
END = "<!-- END draft layout -->"

LENGTH, WIDTH = 70.0, 30.0      # outer building size (draft x, draft y)
WALL_T, WALL_H = 1.0, 8.0       # wall thickness and height
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


def carton_parts(h):
    """Carton box of height h (body + packing tape), length along X, origin at the bottom center."""
    tape = [(0.0, 0.0, h + 0.001, PALLET_L + 0.004, 0.06, 0.002)]          # along the top seam
    tape += [(sx * (PALLET_L / 2 + 0.001), 0.0, h - 0.1, 0.002, 0.06, 0.2)  # down both short sides
             for sx in (-1, 1)]
    return [(0.0, 0.0, h / 2, PALLET_L, PALLET_W, h)], tape


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


def line_geometry(num, top, rows, boxes):
    """Visual boxes per part, collision boxes and pallet slots of one line (world frame).
    boxes: (line, row, shelf, level) -> carton total height or None, from assign_shelf_boxes."""
    n = round((LINE_X1 - LINE_X0) / SHELF_LEN)
    x0, _ = to_world(0, top)
    y0, y1 = to_world(LINE_X0, 0)[1], to_world(LINE_X1, 0)[1]
    depth = SHELF_DEPTH * rows
    vis = {"posts": [], "boards": [], "pallets": [], "cartons": [], "tape": []}
    col, slots = [], []
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
                body, tape = carton_parts(total - PALLET_H)
                vis["cartons"] += [(wx + cx, wy + cy, z + PALLET_H + cz, a, b, c) for cx, cy, cz, a, b, c in body]
                vis["tape"] += [(wx + cx, wy + cy, z + PALLET_H + cz, a, b, c) for cx, cy, cz, a, b, c in tape]
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
    return vis, col, slots


def zone4_geometry():
    """Floor pallets in zone 4 (0.8 m side across the zone depth, 1.2 m along it),
    each with a carton box of a random type from BOX_TOTAL_H."""
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
    vis, col, slots = {"pallets": [], "cartons": [], "tape": []}, [], []
    for i in range(nx):
        wx = x0 + mx + PALLET_W / 2 + i * (PALLET_W + ZONE4_GAP)
        for j in range(ny):
            wy = y0 + my + PALLET_L / 2 + j * (PALLET_L + ZONE4_GAP)
            total = rng.choice(BOX_TOTAL_H)
            body, tape = (rot(p) for p in carton_parts(total - PALLET_H))
            vis["pallets"] += [(wx + cx, wy + cy, cz, a, b, c) for cx, cy, cz, a, b, c in pp]
            vis["cartons"] += [(wx + cx, wy + cy, PALLET_H + cz, a, b, c) for cx, cy, cz, a, b, c in body]
            vis["tape"] += [(wx + cx, wy + cy, PALLET_H + cz, a, b, c) for cx, cy, cz, a, b, c in tape]
            # pallet + box share the footprint: one collision box up to the top of the carton
            col.append((wx, wy, total / 2, PALLET_W, PALLET_L, total))
            slots.append((i + 1, j + 1, 0, wx, wy, PALLET_H, box_name(total)))
    return vis, col, slots


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


def box_sdf(tag, b, pad):
    x, y, z, sx, sy, sz = b
    return (f'{pad}<collision name="{tag}">\n'
            f'{pad}  <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>\n'
            f'{pad}  <geometry><box><size>{sx:g} {sy:g} {sz:g}</size></box></geometry>\n'
            f'{pad}</collision>\n')


def write_model(name, title, vis, col):
    d = MODELS / name
    (d / "meshes").mkdir(parents=True, exist_ok=True)
    visuals = ""
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


def walls():
    hx, hy, t, h = WIDTH / 2, LENGTH / 2, WALL_T, WALL_H
    # name, center x, center y, size x, size y (world frame)
    segs = [
        ("north", 0.0, hy - t / 2, WIDTH, t),
        ("south", 0.0, -hy + t / 2, WIDTH, t),
        ("east", hx - t / 2, 0.0, t, LENGTH - 2 * t),
        ("west", -hx + t / 2, 0.0, t, LENGTH - 2 * t),
    ]
    body = ""
    for n, x, y, sx, sy in segs:
        geom = f"<geometry><box><size>{sx:g} {sy:g} {h:g}</size></box></geometry>"
        body += (
            f'        <collision name="{n}_collision">\n'
            f'          <pose>{x:g} {y:g} {h / 2:g} 0 0 0</pose>\n'
            f'          {geom}\n'
            f'        </collision>\n'
            f'        <visual name="{n}_visual">\n'
            f'          <pose>{x:g} {y:g} {h / 2:g} 0 0 0</pose>\n'
            f'          {geom}\n'
            f'          <material>\n'
            f'            <ambient>0.75 0.75 0.72 1</ambient>\n'
            f'            <diffuse>0.75 0.75 0.72 1</diffuse>\n'
            f'          </material>\n'
            f'        </visual>\n')
    return (
        f'    <!-- Walls: {LENGTH:g} x {WIDTH:g} m outer, {WALL_T:g} m thick, {WALL_H:g} m high -->\n'
        f'    <model name="warehouse_walls">\n'
        f'      <static>true</static>\n'
        f'      <link name="link">\n{body}'
        f'      </link>\n'
        f'    </model>\n')


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
    print("shelf slots:", {(box_name(t) if t else "empty"): c for t, c in counts.items()})
    for num, top, nrows in LINES:
        vis, col, slots = line_geometry(num, top, nrows, shelf_boxes)
        model = write_model(f"warehouse_line_{num}", f"Shelf line {num} (shelves + pallets + boxes)", vis, col)
        incs.append(include(f"shelf_line_{num}", model))
        rows += [f"{num},{r},{s},{lv},{x:.3f},{y:.3f},{z:.3f},{bx}" for r, s, lv, x, y, z, bx in slots]
        print(f"{model}: {sum(len(b) for b in vis.values())} visual boxes in {len(vis)} meshes, "
              f"{len(col)} collision boxes, {len(slots)} pallets")
    vis, col, slots = zone4_geometry()
    model = write_model("warehouse_zone_4", "Zone 4 (floor pallets + carton boxes)", vis, col)
    incs.append(include("pallet_zone_4", model))
    rows += [f"4,{r},{s},{lv},{x:.3f},{y:.3f},{z:.3f},{bx}" for r, s, lv, x, y, z, bx in slots]
    counts = {box_name(t): sum(1 for sl in slots if sl[6] == box_name(t)) for t in BOX_TOTAL_H}
    print(f"{model}: {len(col)} floor pallets with boxes {counts}")
    SLOTS.write_text("\n".join(rows) + "\n")

    s = WORLD.read_text()
    a = s.index(BEGIN)
    a = s.index("\n", a) + 1
    b = s.rindex("\n", 0, s.index(END)) + 1
    block = (walls() + "\n"
             + "    <!-- Shelf lines 1-3 (shelves + pallets merged per line) and zone 4 (pallets + boxes) -->\n"
             + "".join(incs))
    WORLD.write_text(s[:a] + block + s[b:])
    print(f"Wrote walls + {len(incs)} merged models to {WORLD}")
    print(f"Wrote {len(rows) - 1} pallet slots to {SLOTS}")


if __name__ == "__main__":
    main()
