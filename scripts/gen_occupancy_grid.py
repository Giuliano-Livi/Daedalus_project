#!/usr/bin/env python3
"""2D occupancy grid of the warehouse world, from the collision boxes of its SDF models.

Reads gazebo/worlds/warehouse_px4.sdf, follows the model:// includes into gazebo/models,
collects every <box> collision (walls, shelf posts and boards, pallets, carton boxes),
projects it on the XY plane and marks every cell its footprint overlaps as occupied.
Planes and meshes (the floor) are ignored. Only the yaw of the poses is applied.

Grid: CELL x CELL m cells aligned with the building's outer walls, so the world
x in [-15, 15], y in [-35, 35] becomes 30 columns x 70 rows.

Outputs (ros2_ws/src/daedalus_control/maps/):
  warehouse_occupancy.pgm + .yaml  ROS map_server / nav2 map (black = occupied)
  warehouse_occupancy.csv          rows = y cells (south -> north), cols = x cells (west -> east),
                                   1 = occupied, 0 = free; cell (i, j) center is
                                   x = X0 + (j + 0.5) * CELL, y = Y0 + (i + 0.5) * CELL
  warehouse_occupancy.png          preview (if matplotlib is available)

Usage: scripts/gen_occupancy_grid.py [--z-min Z] [--z-max Z]
  --z-min/--z-max: keep only obstacles intersecting that height band (default: all)
"""
import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WORLD = ROOT / "gazebo/worlds/warehouse_px4.sdf"
MODELS = ROOT / "gazebo/models"
OUT = ROOT / "ros2_ws/src/daedalus_control/maps"   # installed with the package: the dev container only sees ros2_ws/src
NAME = "warehouse_occupancy"

CELL = 0.5                       # cell side (m)
X0, X1 = -15.0, 15.0             # world extent of the grid (outer wall faces)
Y0, Y1 = -35.0, 35.0
EPS = 1e-6                       # a footprint only touching a cell edge does not occupy it


def pose(el):
    """(x, y, z, yaw) of the <pose> child of el, zeros if missing."""
    p = el.find("pose")
    v = [float(s) for s in p.text.split()] if p is not None and p.text else [0.0] * 6
    v += [0.0] * (6 - len(v))
    return v[0], v[1], v[2], v[5]


def compose(a, b):
    """Pose b expressed in frame a -> parent frame (translation + yaw)."""
    ax, ay, az, ayaw = a
    bx, by, bz, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, az + bz, ayaw + byaw


def model_boxes(model, parent):
    """World-frame boxes (x, y, z, yaw, sx, sy, sz) of a <model> element and its includes."""
    out = []
    for inc in model.findall("include"):
        uri = inc.find("uri").text.strip()
        sub = ET.parse(MODELS / uri.removeprefix("model://") / "model.sdf").getroot().find("model")
        out += model_boxes(sub, compose(parent, pose(inc)))
    for sub in model.findall("model"):
        out += model_boxes(sub, compose(parent, pose(sub)))
    for link in model.findall("link"):
        lp = compose(parent, pose(link))
        for col in link.findall("collision"):
            box = col.find("geometry/box")
            if box is None:
                continue
            x, y, z, yaw = compose(lp, pose(col))
            sx, sy, sz = (float(s) for s in box.find("size").text.split())
            out.append((x, y, z, yaw, sx, sy, sz))
    return out


def world_boxes():
    world = ET.parse(WORLD).getroot().find("world")
    out = []
    for m in world.findall("model"):
        out += model_boxes(m, pose(m))
    return out


def rasterize(boxes, z_min, z_max):
    nx, ny = round((X1 - X0) / CELL), round((Y1 - Y0) / CELL)
    grid = [[0] * nx for _ in range(ny)]
    for x, y, z, yaw, sx, sy, sz in boxes:
        if z + sz / 2 < z_min or z - sz / 2 > z_max:
            continue
        # axis-aligned footprint (exact for yaw = k * 90 deg, conservative otherwise)
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        hx, hy = (c * sx + s * sy) / 2, (s * sx + c * sy) / 2
        j0 = max(0, math.floor((x - hx - X0) / CELL + EPS))
        j1 = min(nx - 1, math.ceil((x + hx - X0) / CELL - EPS) - 1)
        i0 = max(0, math.floor((y - hy - Y0) / CELL + EPS))
        i1 = min(ny - 1, math.ceil((y + hy - Y0) / CELL - EPS) - 1)
        for i in range(i0, i1 + 1):
            for j in range(j0, j1 + 1):
                grid[i][j] = 1
    return grid


def write_outputs(grid):
    OUT.mkdir(parents=True, exist_ok=True)
    ny, nx = len(grid), len(grid[0])
    (OUT / f"{NAME}.csv").write_text("\n".join(",".join(map(str, r)) for r in grid) + "\n")
    # PGM: first image row = north (max y); 0 = occupied, 254 = free
    pix = bytes(0 if v else 254 for r in reversed(grid) for v in r)
    (OUT / f"{NAME}.pgm").write_bytes(f"P5\n{nx} {ny}\n255\n".encode() + pix)
    (OUT / f"{NAME}.yaml").write_text(
        f"image: {NAME}.pgm\n"
        f"mode: trinary\n"
        f"resolution: {CELL}\n"
        f"origin: [{X0}, {Y0}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(figsize=(5, 10))
    ax.imshow(grid, origin="lower", cmap="gray_r", extent=(X0, X1, Y0, Y1), vmin=0, vmax=1.6)
    ax.set_xticks(range(int(X0), int(X1) + 1, 5))
    ax.set_yticks(range(int(Y0), int(Y1) + 1, 5))
    ax.set_xticks([X0 + k * CELL for k in range(nx + 1)], minor=True)
    ax.set_yticks([Y0 + k * CELL for k in range(ny + 1)], minor=True)
    ax.grid(which="minor", color="0.75", linewidth=0.3)
    ax.tick_params(which="minor", length=0)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{NAME} ({CELL:g} m cells)")
    fig.tight_layout()
    fig.savefig(OUT / f"{NAME}.png", dpi=150)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--z-min", type=float, default=-math.inf)
    ap.add_argument("--z-max", type=float, default=math.inf)
    a = ap.parse_args()
    boxes = world_boxes()
    grid = rasterize(boxes, a.z_min, a.z_max)
    write_outputs(grid)
    occ = sum(map(sum, grid))
    total = len(grid) * len(grid[0])
    print(f"{len(boxes)} collision boxes -> {len(grid[0])} x {len(grid)} grid, "
          f"{occ} occupied / {total - occ} free cells")
    print(f"Wrote {OUT}/{NAME}.{{pgm,yaml,csv,png}}")


if __name__ == "__main__":
    main()
