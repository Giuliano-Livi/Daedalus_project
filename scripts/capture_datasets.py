#!/usr/bin/env python3
"""Shelf image datasets for the computer-vision tasks, captured in Gazebo with the ZED 2i noise model.

  freespot   672 x 376 (the drone camera), 4.6 m from a shelf face, 3 m high: one block of
             3 bays x 3 levels (9 pallet spots) per image, every block of 3 neighbouring bays
             (stride 1) of the 5 shelf faces of lines 1-3
  inventory  2208 x 1242 (ZED 2i HD2K, needed to resolve the 1.16 mm ITF-14 modules), 0.5 m from
             the label of one pallet spot (empty spots: from the pallet front), every shelf spot

Output (<out>/dataset_<task>/):
  images/<name>.png   colour with camera noise (camera_noise node)
  depth/<name>.png    depth with ZED noise, 16-bit PNG in mm, 0 = no depth; same name as the image
  annotations.csv     ground truth per image (see the header), camera.json (intrinsics)
Each camera pose gets a small random jitter (fixed seed); the annotations use the jittered pose.

Usage (scripts/capture_datasets.sh runs everything in the containers):
  capture_datasets.py world <task> <world.sdf>    capture world: warehouse + one movable camera
  capture_datasets.py capture <task> <out_dir>    inside the ROS container, with bridge + noise
"""
import csv
import json
import math
import os
import random
import sys
import time
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
import gen_warehouse_layout as layout          # noqa: E402
import gen_warehouse_textures as textures      # noqa: E402

WORLD_NAME = "warehouse_px4"
CAMERA = "dataset_camera"
HFOV = 1.77                                    # ZED 2i, same as gazebo/models/x500_depth
TASKS = {
    # resolution, distance from the target plane, jitter (position m, angles rad), noise overrides
    # (jitter: the 6 m tall loads of the top level reach the top edge of the frame, pitch kept small)
    "freespot": dict(size=(672, 376), dist=4.6, z=3.0, jitter=(0.10, math.radians(1.0)), noise={}),
    "inventory": dict(size=(2208, 1242), dist=0.5, jitter=(0.03, math.radians(2.0)),
                      # pixel sizes of the noise model scaled by 2208 / 672
                      noise={"depth.spatial_cell": 79, "depth.blob_cell": 39, "depth.edge_width": 3}),
}
SEED = 2026
LEVEL_TOPS = (0.0, 2.0, 4.0)                   # floor / board tops
LEVEL_CLEAR = (1.96, 1.96, 2.0)                # clear height of each level (top level: tallest load)


# ---------------------------------------------------------------- geometry
def rot(roll, pitch, yaw):
    """Rotation matrix of a Gazebo pose (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)   # w, x, y, z


def intrinsics(task):
    w, h = TASKS[task]["size"]
    f = (w / 2) / math.tan(HFOV / 2)
    return w, h, f, w / 2, h / 2


def project(pose, pts, task):
    """Pixel bounding box [u0, v0, u1, v1] of world points (clipped to the image), or None.
    Gazebo camera: looks along +X of its frame, image right = -Y, image down = -Z."""
    w, h, f, cx, cy = intrinsics(task)
    x, y, z, roll, pitch, yaw = pose
    r = rot(roll, pitch, yaw)
    us, vs = [], []
    for p in pts:
        d = (p[0] - x, p[1] - y, p[2] - z)
        q = [sum(r[i][k] * d[i] for i in range(3)) for k in range(3)]   # R^T d
        if q[0] <= 0.05:
            return None
        us.append(cx - f * q[1] / q[0])
        vs.append(cy - f * q[2] / q[0])
    u0, u1 = max(0.0, min(us)), min(float(w), max(us))
    v0, v1 = max(0.0, min(vs)), min(float(h), max(vs))
    if u1 <= u0 or v1 <= v0:
        return None
    return [round(u0), round(v0), round(u1), round(v1)]


# ---------------------------------------------------------------- layout
def faces():
    """Shelf faces towards an aisle: (line, side, face_x); side +1 = face looking to +X."""
    out = []
    for num, top, nrows in layout.LINES:
        x0 = top - layout.WIDTH / 2
        x1 = x0 + layout.SHELF_DEPTH * nrows
        if nrows == 2:
            out.append((num, -1, x0))
        out.append((num, 1, x1))
    return out


def slots():
    """Shelf pallet spots of lines 1-3 with the side they face."""
    face_x = {(n, s): fx for n, s, fx in faces()}
    out = []
    for r in csv.DictReader(open(ROOT / "gazebo/worlds/pallet_slots.csv")):
        n = int(r["line"])
        if n > 3:
            continue
        x, y = float(r["x"]), float(r["y"])
        side = 1 if (n, -1) not in face_x else (1 if x > (face_x[(n, -1)] + face_x[(n, 1)]) / 2 else -1)
        out.append(dict(line=n, side=side, face_x=face_x[(n, side)], shelf=int(r["shelf"]),
                        level=int(r["level"]), x=x, y=y, z_top=float(r["z_top"]),
                        box=r["box"] or "", occupied=bool(r["box"])))
    return out


def labels():
    """Label quads of lines 1-3 from the meshes: centre, outward normal, corners, ITF-14 code."""
    out = []
    for n in (1, 2, 3):
        name = f"warehouse_line_{n}"
        v, vn = [], []
        for line in open(layout.MODELS / name / "meshes" / "labels.obj"):
            if line.startswith("v "):
                v.append(tuple(map(float, line.split()[1:4])))
            elif line.startswith("vn "):
                vn.append(tuple(map(float, line.split()[1:4])))
        for k in range(len(v) // 4):
            quad = v[4 * k:4 * k + 4]
            c = tuple(sum(p[i] for p in quad) / 4 for i in range(3))
            out.append(dict(line=n, centre=c, normal=vn[k], corners=quad, code=barcode(name, k)))
    return out


def barcode(model, k):
    """ITF-14 printed on label k of a model (same random sequence as make_label_atlas)."""
    rng = random.Random(zlib.crc32(f"{model}-{k}".encode()))
    rng.randint(50, 90)
    for _ in range(4):
        rng.randint(30, 110)
    for _ in range(2):
        rng.randint(30, 80)
    gtin12 = "8" + str(rng.randint(0, 3)) + "".join(str(rng.randint(0, 9)) for _ in range(10))
    return textures.itf14("1" + gtin12)[0]


def spot_rect(s):
    """Corners of a spot's opening on the shelf face plane (between posts and boards)."""
    p = layout.POST
    y0, y1 = s["y"] - layout.SHELF_LEN / 2 + p, s["y"] + layout.SHELF_LEN / 2 - p
    z0 = LEVEL_TOPS[s["level"]]
    z1 = z0 + LEVEL_CLEAR[s["level"]]
    fx = s["face_x"]
    return [(fx, y0, z0), (fx, y1, z0), (fx, y1, z1), (fx, y0, z1)]


def jitter(rng, task, x, y, z, yaw):
    dp, da = TASKS[task]["jitter"]
    u = lambda a: rng.uniform(-a, a)
    return (round(x + u(dp), 4), round(y + u(dp), 4), round(z + u(dp), 4),
            0.0, round(u(da), 5), round(yaw + u(da), 5))


def side_name(side):
    return "east" if side > 0 else "west"      # world +X = east (ENU)


def plan(task):
    """[(name, pose, annotation dict)] for every image of a task."""
    rng = random.Random(f"{SEED}-{task}")
    all_slots = slots()
    items = []
    if task == "freespot":
        for n, side, fx in faces():
            face = {(s["shelf"], s["level"]): s for s in all_slots if s["line"] == n and s["side"] == side}
            shelves = sorted({k[0] for k in face})
            for i in range(len(shelves) - 2):
                block = shelves[i:i + 3]
                yc = sum(face[(b, 0)]["y"] for b in block) / 3
                pose = jitter(rng, task, fx + side * TASKS[task]["dist"], yc, TASKS[task]["z"],
                              math.pi if side > 0 else 0.0)
                spots = []
                for b in block:
                    for lv in (0, 1, 2):
                        s = face[(b, lv)]
                        spots.append(dict(shelf=b, level=lv, occupied=s["occupied"],
                                          bbox=project(pose, spot_rect(s), task)))
                # reading order: top level first, then left to right in the image
                spots.sort(key=lambda d: (-d["level"], d["bbox"][0] if d["bbox"] else 0))
                items.append((f"line{n}_{side_name(side)}_shelves{block[0]:02d}-{block[2]:02d}", pose,
                              dict(line=n, face=side_name(side), spots=spots)))
    else:
        labs = labels()
        for s in all_slots:
            lab = None
            for l in labs:
                c = l["centre"]
                if (l["line"] == s["line"] and abs(c[1] - s["y"]) < 0.8 and abs(c[0] - s["x"]) < 1.0
                        and s["z_top"] < c[2] < s["z_top"] + 2.0):
                    lab = l
            side = s["side"]
            if lab:
                c, nrm = lab["centre"], lab["normal"]
                d = TASKS[task]["dist"]
                pose = jitter(rng, task, c[0] + nrm[0] * d, c[1] + nrm[1] * d, c[2] + nrm[2] * d,
                              math.atan2(-nrm[1], -nrm[0]))
            else:   # empty spot: look at the pallet front, where a label would be
                front = s["x"] + side * layout.PALLET_L / 2
                pose = jitter(rng, task, front + side * TASKS[task]["dist"], s["y"], s["z_top"] + 0.35,
                              math.pi if side > 0 else 0.0)
            name = f"line{s['line']}_{side_name(side)}_shelf{s['shelf']:02d}_level{s['level']}"
            items.append((name, pose, dict(
                line=s["line"], face=side_name(side), shelf=s["shelf"], level=s["level"],
                occupied=s["occupied"], barcode=lab["code"] if lab else "",
                label_bbox=project(pose, lab["corners"], task) if lab else None,
                spot_bbox=project(pose, spot_rect(s), task))))
    return items


# ---------------------------------------------------------------- capture world
def write_world(task, path):
    w, h = TASKS[task]["size"]
    s = (ROOT / "gazebo/worlds/warehouse_px4.sdf").read_text()
    plugins = """
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>"""
    sensor = lambda kind, typ, fmt, near: f"""
        <sensor name="{kind}" type="{typ}">
          <topic>dataset/camera/{kind}/image</topic>
          <camera><horizontal_fov>{HFOV}</horizontal_fov><image><width>{w}</width><height>{h}</height><format>{fmt}</format></image><clip><near>{near}</near><far>100</far></clip></camera>
          <always_on>1</always_on><update_rate>5</update_rate>
        </sensor>"""
    x, y, z, roll, pitch, yaw = plan(task)[0][1]
    cam = f"""
    <model name="{CAMERA}">
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
      <link name="link">
        <gravity>false</gravity>
        <inertial><mass>0.1</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz></inertia></inertial>
        {sensor("color", "camera", "R8G8B8", 0.05)}{sensor("depth", "depth_camera", "R_FLOAT32", 0.05)}
      </link>
    </model>"""
    s = s.replace(f'<world name="{WORLD_NAME}">', f'<world name="{WORLD_NAME}">' + plugins, 1)
    s = s.replace("\n  </world>", cam + "\n  </world>", 1)
    Path(path).write_text(s)
    print(f"{path}: {task} camera {w}x{h}, {len(plan(task))} poses")


# ---------------------------------------------------------------- capture (ROS container)
def capture(task, out):
    import cv2
    import numpy as np
    import rclpy
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node as GzNode
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    items = plan(task)
    root = Path(out) / f"dataset_{task}"
    (root / "images").mkdir(parents=True, exist_ok=True)
    (root / "depth").mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Node(f"capture_{task}")
    latest = {}
    stamp = lambda m: m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
    node.create_subscription(Image, "/dataset/camera/color/noisy", lambda m: latest.__setitem__("color", m), 2)
    node.create_subscription(Image, "/dataset/camera/depth/noisy", lambda m: latest.__setitem__("depth", m), 2)
    gz = GzNode()
    sim = [0.0]
    gz.subscribe(Clock, f"/world/{WORLD_NAME}/clock",
                 lambda m: sim.__setitem__(0, m.sim.sec + 1e-9 * m.sim.nsec))

    t0 = time.time()
    while time.time() < t0 + 60 and len(latest) < 2:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(latest) < 2:
        sys.exit("no noisy images: bridge / camera_noise not running?")

    rows, done = [], 0
    for name, pose, ann in items:
        x, y, z, roll, pitch, yaw = pose
        req = Pose()
        req.name = CAMERA
        req.position.x, req.position.y, req.position.z = x, y, z
        qw, qx, qy, qz = quat(roll, pitch, yaw)
        req.orientation.w, req.orientation.x, req.orientation.y, req.orientation.z = qw, qx, qy, qz
        for _ in range(10):            # the first requests can come before the service is discovered
            ok, rep = gz.request(f"/world/{WORLD_NAME}/set_pose", req, Pose, Boolean, 3000)
            if ok and rep.data:
                break
            time.sleep(1.0)
        else:
            sys.exit(f"set_pose failed for {name}")
        # first frames rendered after the move (+ 2 frames of margin for the noise filters)
        after = sim[0] + 0.45
        t1 = time.time()
        while time.time() < t1 + 30 and not all(stamp(latest[k]) >= after for k in ("color", "depth")):
            rclpy.spin_once(node, timeout_sec=0.02)
        c, d = latest["color"], latest["depth"]
        rgb = np.frombuffer(bytes(c.data), np.uint8).reshape(c.height, c.step)[:, :3 * c.width]
        rgb = rgb.reshape(c.height, c.width, 3)
        cv2.imwrite(str(root / "images" / f"{name}.png"), rgb[:, :, ::-1] if c.encoding == "rgb8" else rgb)
        if d.encoding == "16UC1":
            mm = np.frombuffer(bytes(d.data), np.uint16).reshape(d.height, d.step // 2)[:, :d.width]
        else:
            z_m = np.frombuffer(bytes(d.data), np.float32).reshape(d.height, d.step // 4)[:, :d.width]
            mm = np.where(np.isfinite(z_m), np.clip(np.round(z_m * 1000), 0, 65535), 0).astype(np.uint16)
        cv2.imwrite(str(root / "depth" / f"{name}.png"), mm)
        valid = float((mm > 0).mean())
        rows.append((name, pose, ann, valid))
        done += 1
        if done % 25 == 0 or done == len(items):
            print(f"{task}: {done}/{len(items)} ({time.time() - t0:.0f} s)", flush=True)

    w, h, f, cx, cy = intrinsics(task)
    json.dump({"width": w, "height": h, "K": [f, 0, cx, 0, f, cy, 0, 0, 1], "distortion": [0] * 5,
               "hfov_rad": HFOV, "depth": "16-bit PNG, millimetres, 0 = no depth (ZED 2i noise model)",
               "camera_frame": "Gazebo: x forward, y left, z up; pose = world x y z roll pitch yaw (rad)"},
              open(root / "camera.json", "w"), indent=1)
    with open(root / "annotations.csv", "w", newline="") as fh:
        wr = csv.writer(fh)
        if task == "freespot":
            wr.writerow(["image", "line", "face", "spot", "shelf", "level", "occupied", "u0", "v0", "u1", "v1",
                         "cam_x", "cam_y", "cam_z", "cam_roll", "cam_pitch", "cam_yaw", "depth_valid"])
            for name, pose, ann, valid in rows:
                for k, s in enumerate(ann["spots"]):
                    wr.writerow([name, ann["line"], ann["face"], k, s["shelf"], s["level"], int(s["occupied"]),
                                 *(s["bbox"] or ["", "", "", ""]), *pose, f"{valid:.3f}"])
        else:
            wr.writerow(["image", "line", "face", "shelf", "level", "occupied", "barcode",
                         "label_u0", "label_v0", "label_u1", "label_v1", "spot_u0", "spot_v0", "spot_u1", "spot_v1",
                         "cam_x", "cam_y", "cam_z", "cam_roll", "cam_pitch", "cam_yaw", "depth_valid"])
            for name, pose, ann, valid in rows:
                wr.writerow([name, ann["line"], ann["face"], ann["shelf"], ann["level"], int(ann["occupied"]),
                             ann["barcode"], *(ann["label_bbox"] or ["", "", "", ""]),
                             *(ann["spot_bbox"] or ["", "", "", ""]), *pose, f"{valid:.3f}"])
    print(f"{task}: wrote {root}", flush=True)
    os._exit(0)     # the gz-transport node aborts in its destructor at interpreter exit


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "world":
        write_world(sys.argv[2], sys.argv[3])
    elif len(sys.argv) == 4 and sys.argv[1] == "capture":
        capture(sys.argv[2], sys.argv[3])
    elif len(sys.argv) == 3 and sys.argv[1] == "noise":
        print(" ".join(f"-p {k}:={v}" for k, v in TASKS[sys.argv[2]]["noise"].items()))
    else:
        sys.exit(__doc__)
