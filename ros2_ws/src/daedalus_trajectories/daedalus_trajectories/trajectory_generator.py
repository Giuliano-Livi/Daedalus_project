import rclpy
import math
import heapq
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from daedalus_interfaces.msg import FlightSetpoint


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        self.x_setpoint = 0
        self.current_position = [0,0,0]
        self.target_pos = [[0,54.5,3], [7.6, 54.5, 3], [7.6, -4.5, 3], [16.2, -4.5, 3], [16.2, 54.5, 3], [7.6, 54.5, 3], [7.6, -4.5, 3], [0,0,3]] # [16.2, 54.5, 3], [0,54.5,3], [0,0,3], [7.6, -4.5, 3], [0,0,3]]
        self.des_yaw = math.pi/2

        # 2D occupancy grid from scripts/gen_occupancy_grid.py: 1 = occupied
        self.grid = np.loadtxt(
            os.path.join(get_package_share_directory('daedalus_trajectories'), 'maps', 'warehouse_occupancy.csv'),
            delimiter=',', dtype=np.uint8)
        self.grid_origin = (-15.0, -35.0)   # world x, y of the grid's south-west corner
        self.grid_res = 0.5                 # cell side (m)
        self.spawn_xy = (-10.2, -25.0)      # drone spawn in world
        self.turn_cost = 2.0                # A* extra cost (m) for every change of direction, 0 = shortest path
        self.inflation = 4                  # safety margin: cells added around every obstacle, 0 = none
        self.center_dist = 4.5              # m: closer than this to an obstacle, a step costs extra (>= half the widest lane)
        self.center_weight = 0.3            # extra cost per step for each m closer than center_dist, 0 = off

        # clearance: distance (m) from every cell to the nearest obstacle cell of the raw map,
        # computed one row at a time (all obstacle cells vs the cells of that row)
        oi, oj = np.nonzero(self.grid)
        cols = np.arange(self.grid.shape[1])
        self.clearance = np.array([np.hypot(i - oi[:, None], cols - oj[:, None]).min(axis=0)
                                   for i in range(self.grid.shape[0])]) * self.grid_res

        # inflate: a cell is occupied if any cell within `inflation` cells of it is
        r, (h, w) = self.inflation, self.grid.shape
        padded = np.pad(self.grid, r)             # enlarging the map in each direction adding 0
        # slides the map in each direction and then stacks them together and takes the result
        # in this way we have enlarged of 1 line all the 1s
        self.grid = np.max([padded[r + di:r + di + h, r + dj:r + dj + w]
                            for di in range(-r, r + 1) for dj in range(-r, r + 1)], axis=0)

        # smooth trajectory: corners rounded instead of stopping at every waypoint
        self.blend_radius = 1.5                  # m: a corner is rounded from up to this far before to after it
        self.stop_angle = math.radians(120)      # turns sharper than this (path reversals) stop at the corner
        self.yaw_lead = 1.0                      # m: face the path direction this far ahead (turn before the corner)
        self.yaw_rate_max = math.radians(25)     # below PX4's MC_YAWRATE_MAX (30 deg/s, docker-compose.yml)
        self.ds = 0.02                           # m: resolution of the path used to build the speed profile

        self.pos_traj = []
        self.vel_traj = []
        self.acc_traj = []
        self.yaw_traj = []
        self.yaw_rate_traj = []
        self.step_of_traj = 0
        self.trajectory_generated = False



        self.setpoint_pub = self.create_publisher(
            FlightSetpoint,
            '/drone/setpoint',
            10
        )


        # 10Hz loop
        self.create_timer(
            0.1,
            self.publish
        )

    # callbacks
    def publish(self):
        setpoint = FlightSetpoint()
        if not self.trajectory_generated:
            self.generate_trajectory()
        if self.step_of_traj < len(self.pos_traj):
            x, y, z = self.pos_traj[self.step_of_traj]
            vx, vy, vz = self.vel_traj[self.step_of_traj]
            ax, ay, az = self.acc_traj[self.step_of_traj]
            setpoint.yaw = self.yaw_traj[self.step_of_traj]
            setpoint.yaw_rate = self.yaw_rate_traj[self.step_of_traj]
            self.step_of_traj += 1
            setpoint.position = Point(x=float(x), y=float(y), z=float(z))
            setpoint.velocity = Vector3(x=float(vx), y=float(vy), z=float(vz))
            setpoint.acceleration = Vector3(x=float(ax), y=float(ay), z=float(az))
            self.setpoint_pub.publish(setpoint)
        else:
            x, y, z = self.pos_traj[-1]
            vx, vy, vz = self.vel_traj[-1]
            ax, ay, az = self.acc_traj[-1]
            self.step_of_traj += 1
            setpoint.position = Point(x=float(x), y=float(y), z=float(z))
            setpoint.velocity = Vector3(x=float(vx), y=float(vy), z=float(vz))
            setpoint.acceleration = Vector3(x=float(ax), y=float(ay), z=float(az))
            setpoint.yaw = self.yaw_traj[-1]
            setpoint.yaw_rate = 0.0
            self.setpoint_pub.publish(setpoint)


    # functions
    def quintic(self, x0, v0, a0, xf, vf, af, T):
        c0, c1, c2 =  x0, v0, a0/2
        A = np.array([[T**3,    T**4,     T**5],
                    [3*T**2,  4*T**3,   5*T**4],
                    [6*T,     12*T**2,  20*T**3]])
        b = np.array([xf - (c0 + c1*T + c2*T**2),
                    vf - (c1 + 2*c2*T),
                    af - 2*c2])
        c3, c4, c5 = np.linalg.solve(A, b)
        return np.array([c0, c1, c2, c3, c4, c5])

    def sample(self, c, t):
        x = sum(c[i] * t**i for i in range(6))
        v = sum(i * c[i] * t**(i-1) for i in range(1, 6))
        a = sum(i*(i-1) * c[i] * t**(i-2) for i in range(2, 6))
        return x, v, a

    def setpoint_to_cell(self, p):
        # converts the target in grid dimensions and in drone's world coordinates
        x, y = p[0] + self.spawn_xy[0], p[1] + self.spawn_xy[1]
        return (int((y - self.grid_origin[1]) // self.grid_res),
                int((x - self.grid_origin[0]) // self.grid_res))

    def cell_to_setpoint(self, c, z):
        # grid cell (i, j) -> setpoint at the cell center
        x = self.grid_origin[0] + (c[1] + 0.5) * self.grid_res - self.spawn_xy[0]
        y = self.grid_origin[1] + (c[0] + 0.5) * self.grid_res - self.spawn_xy[1]
        return [x, y, z]

    def is_free(self, c):
        return 0 <= c[0] < self.grid.shape[0] and 0 <= c[1] < self.grid.shape[1] and not self.grid[c]

    def astar(self, start, goal):
        # cheapest 4-connected path of cells from start to goal, None if unreachable.
        # cost = length + turn_cost for every change of direction + a penalty for cells close
        # to obstacles (keeps the path in the middle of the lanes), so the state is
        # (cell, direction of the move that reached it); (0, 0) = no move yet
        if not (self.is_free(start) and self.is_free(goal)):
            return None
        moves = [(1,0),(-1,0),(0,1),(0,-1)]
        # h guesses how far a cell is from the goal
        # is activated only when A* runs that cell
        # it is a 1 line function
        h = lambda c: math.hypot(c[0] - goal[0], c[1] - goal[1])
        s0 = (start, (0, 0))     # state initialization, point + previous cell to evaluate the direction change
        g = {s0: 0.0}            # notebook with the best cost found at the moment to reach the state
        parent = {}              # annotates the state you came from to form the final path
        open_ = [(h(start), s0)] # states to be explored
        closed = set()           # states already explored, set() creates an empty set, list without duplicates
        while open_:
            _, s = heapq.heappop(open_)
            cur, d = s           # current and direction of the state
            
            # if the goal is reached follow the parent notebook and append the setpoints
            if cur == goal:
                path = [cur]
                while s in parent:
                    s = parent[s]
                    path.append(s[0])
                return path[::-1]
            
            # is the state is already explored add it to closed, happens when a state is added to the list twice
            if s in closed:
                continue
            closed.add(s)

            # try the moves 
            for m in moves:
                di, dj = m
                n = (cur[0] + di, cur[1] + dj)
                if not self.is_free(n):
                    continue
                turn = self.turn_cost if d != (0, 0) and m != d else 0.0
                near = max(0.0, self.center_dist - self.clearance[n])   # 0 when far enough from obstacles
                ng = g[s] + math.hypot(di, dj) + turn + self.center_weight * near
                ns = (n, m)
                if ng < g.get(ns, math.inf):
                    g[ns], parent[ns] = ng, s
                    heapq.heappush(open_, (ng + h(n), ns))
        return None

    def compact(self, path):
        # keep only the cells where the direction changes: one segment per straight line
        out = path[:1]
        for a, b, c in zip(path, path[1:], path[2:]):
            if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
                out.append(b)
        if len(path) > 1:
            out.append(path[-1])
        return out

    def plan_path(self, p0, p1):
        # A* from setpoint p0 to p1: exact p0, the corners of the path at p1's height, exact p1
        cells = self.astar(self.setpoint_to_cell(p0), self.setpoint_to_cell(p1))
        if cells is None:
            raise RuntimeError(f'A*: no path from {p0} to {p1} (occupied or outside the map?)')
        corners = self.compact(cells)
        return [p0] + [self.cell_to_setpoint(c, p1[2]) for c in corners[1:-1]] + [p1]

    def smooth_path(self, waypoints, stops):
        # dense path (one point every ~ds m) through the waypoints, each corner replaced by a
        # quadratic Bezier from blend_radius before to blend_radius after it (control point = the
        # corner, so the path stays tangent to both segments and passes ~0.35 * radius from a 90 deg
        # corner); returns points, curvature and stop flags (start, end, `stops` indices of
        # waypoints, reversals sharper than stop_angle)
        pts, stp = [np.asarray(waypoints[0], float)], [True]
        for k, p in enumerate(waypoints[1:], 1):
            p = np.asarray(p, float)
            if np.linalg.norm(p - pts[-1]) > 1e-6:      # skip repeated waypoints
                pts.append(p)
                stp.append(k in stops)
            else:
                stp[-1] = stp[-1] or k in stops
        stp[-1] = True
        n = len(pts)
        r = [0.0] * n                                   # blend radius of every corner, 0 = sharp
        for i in range(1, n - 1):
            d_in, d_out = pts[i] - pts[i - 1], pts[i + 1] - pts[i]
            l_in, l_out = np.linalg.norm(d_in), np.linalg.norm(d_out)
            angle = math.acos(np.clip(d_in @ d_out / (l_in * l_out), -1.0, 1.0))
            if stp[i] or angle > self.stop_angle:
                stp[i] = True
            elif angle > 1e-3:
                r[i] = min(self.blend_radius, 0.45 * l_in, 0.45 * l_out)   # blends never overlap

        P, K, S = [pts[0]], [0.0], [True]
        for i in range(1, n):
            d = (pts[i] - pts[i - 1]) / np.linalg.norm(pts[i] - pts[i - 1])
            start, end = pts[i - 1] + d * r[i - 1], pts[i] - d * r[i]       # straight part
            m = max(1, math.ceil(np.linalg.norm(end - start) / self.ds))
            for j in range(1, m + 1):
                P.append(start + (end - start) * j / m)
                K.append(0.0)
                S.append(False)
            if r[i] == 0.0:                             # sharp: stop at the corner (or the end)
                S[-1] = stp[i]
                continue
            A, C = end, pts[i]
            B = C + (pts[i + 1] - C) / np.linalg.norm(pts[i + 1] - C) * r[i]
            m = max(2, math.ceil(2 * r[i] / self.ds))
            for j in range(1, m + 1):
                t = j / m
                d1 = 2 * (1 - t) * (C - A) + 2 * t * (B - C)
                d2 = 2 * (A - 2 * C + B)
                P.append((1 - t) ** 2 * A + 2 * (1 - t) * t * C + t ** 2 * B)
                K.append(np.linalg.norm(np.cross(d1, d2)) / np.linalg.norm(d1) ** 3)
                S.append(False)
        return np.array(P), np.array(K), np.array(S)

    def speed_profile(self, P, K, S, v_max, a_max):
        # speed along the dense path: v_max on straights, limited on curves by the lateral
        # acceleration (v^2 k <= a_max) and the yaw rate (v k <= yaw_rate_max), 0 at the stops;
        # then forward / backward passes so |dv/dt| <= a_max
        ds = np.linalg.norm(np.diff(P, axis=0), axis=1)
        with np.errstate(divide='ignore'):
            v = np.minimum(v_max, np.minimum(np.sqrt(a_max / K), self.yaw_rate_max / K))
        v[S] = 0.0
        for i in range(1, len(v)):
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_max * ds[i - 1]))
        for i in range(len(v) - 2, -1, -1):
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_max * ds[i]))
        return v, ds

    def time_piece(self, P, v, ds, a_max, dt):
        # one rest-to-rest piece sampled every dt: positions, velocities, accelerations and the
        # yaw to face (direction of the path yaw_lead m ahead, NaN where it is vertical)
        vs = v[:-1] + v[1:]
        dts = np.where(vs > 1e-6, 2 * ds / np.maximum(vs, 1e-6), np.sqrt(4 * ds / a_max))
        t = np.concatenate([[0.0], np.cumsum(dts)])
        tang = np.diff(P, axis=0) / ds[:, None]
        tang = np.vstack([tang, tang[-1:]])
        horiz = np.hypot(tang[:, 0], tang[:, 1])
        head = np.where(horiz > 0.1, np.arctan2(tang[:, 1], tang[:, 0]), np.nan)
        s = np.concatenate([[0.0], np.cumsum(ds)])
        ahead = head[np.minimum(np.searchsorted(s, s + self.yaw_lead), len(s) - 1)]
        head = np.where(np.isnan(ahead), head, ahead)
        valid = np.flatnonzero(~np.isnan(head))
        if len(valid):                                  # fill vertical parts, then unwrap
            head = np.interp(np.arange(len(head)), valid, np.unwrap(head[valid]))
        tt = np.append(np.arange(dt, t[-1] - 1e-9, dt), t[-1])
        k = np.clip(np.searchsorted(t, tt) - 1, 0, len(t) - 2)
        f = ((tt - t[k]) / np.maximum(t[k + 1] - t[k], 1e-9))[:, None]
        pos = P[k] * (1 - f) + P[k + 1] * f
        vel = tang[k] * (v[k] * (1 - f[:, 0]) + v[k + 1] * f[:, 0])[:, None]
        acc = np.gradient(vel, dt, axis=0) if len(tt) > 1 else np.zeros_like(vel)
        yaw = head[k] * (1 - f[:, 0]) + head[k + 1] * f[:, 0] if len(valid) else np.full(len(tt), np.nan)
        return pos, vel, acc, yaw

    def append_setpoint(self, p, v, a, yaw, yaw_rate):
        self.pos_traj.append(list(p))
        self.vel_traj.append(list(v))
        self.acc_traj.append(list(a))
        self.yaw_traj.append(math.atan2(math.sin(yaw), math.cos(yaw)))
        self.yaw_rate_traj.append(yaw_rate)

    def generate_trajectory(self, v_max=1.0, a_max=1.0, dt=0.1):
        # vertical takeoff to the first target's height, then A* through every target
        takeoff = [0, 0, self.target_pos[0][2]]
        targets = [takeoff] + self.target_pos
        waypoints = [self.current_position, takeoff]
        # pairs the targets in couples (T0,T1), (T1,T2) ...
        for p0, p1 in zip(targets[:-1], targets[1:]):
            waypoints += self.plan_path(p0, p1)[1:] # [1:] to avoid repeating the takeoff point twice
        self.get_logger().info(f'A*: {len(waypoints)} waypoints: {[[round(v, 2) for v in p] for p in waypoints]}')

        # one smooth trajectory instead of stopping at every waypoint: rounded corners, speed kept
        # through them; it stops only after the vertical takeoff, at path reversals and at the end
        P, K, S = self.smooth_path(waypoints, stops={1})
        v, ds = self.speed_profile(P, K, S, v_max, a_max)
        stops = np.flatnonzero(S)
        yaw, step = self.des_yaw, self.yaw_rate_max * dt
        for i0, i1 in zip(stops[:-1], stops[1:]):
            pos, vel, acc, head = self.time_piece(P[i0:i1 + 1], v[i0:i1 + 1], ds[i0:i1], a_max, dt)
            first = head[~np.isnan(head)]
            if len(first):                              # at a stop: turn in place to the new direction
                turn = math.atan2(math.sin(first[0] - yaw), math.cos(first[0] - yaw))
                n = math.ceil(abs(turn) / (0.6 * step))  # cosine profile: peak rate ~0.94 * yaw_rate_max
                for j in range(1, n + 1):
                    y = yaw + turn * (1 - math.cos(math.pi * j / n)) / 2
                    rate = turn * math.pi / (2 * n * dt) * math.sin(math.pi * j / n)
                    self.append_setpoint(P[i0], [0, 0, 0], [0, 0, 0], y, rate)
                yaw += turn
            for k in range(len(pos)):                   # fly, facing yaw_lead m ahead (rate limited)
                if not np.isnan(head[k]):
                    err = math.atan2(math.sin(head[k] - yaw), math.cos(head[k] - yaw))
                    d = max(-step, min(step, err))
                else:
                    d = 0.0
                yaw += d
                self.append_setpoint(pos[k], vel[k], acc[k], yaw, d / dt)
        self.des_yaw = yaw
        self.get_logger().info(
            f'smooth trajectory: {len(self.pos_traj) * dt:.1f} s, {len(stops) - 2} stops '
            f'(takeoff + reversals), {np.sum(ds):.1f} m')
        self.trajectory_generated = True




def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
