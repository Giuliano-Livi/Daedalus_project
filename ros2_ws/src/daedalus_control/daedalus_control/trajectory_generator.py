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
        self.target_pos = [15,45,3]
        self.des_yaw = math.pi/2

        # 2D occupancy grid from scripts/gen_occupancy_grid.py: 1 = occupied
        self.grid = np.loadtxt(
            os.path.join(get_package_share_directory('daedalus_control'), 'maps', 'warehouse_occupancy.csv'),
            delimiter=',', dtype=np.uint8)
        self.grid_origin = (-15.0, -35.0)   # world x, y of the grid's south-west corner
        self.grid_res = 0.5                 # cell side (m)
        self.spawn_xy = (-10.2, -25.0)      # drone spawn in world
        self.turn_cost = 2.0                # A* extra cost (m) for every change of direction, 0 = shortest path
        self.inflation = 4                  # safety margin: cells added around every obstacle, 0 = none
        self.center_dist = 4.5              # m: closer than this to an obstacle, a step costs extra (>= half the widest lane)
        self.center_weight = 0.5            # extra cost per step for each m closer than center_dist, 0 = off

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

        self.pos_traj = []
        self.vel_traj = []
        self.acc_traj = []
        self.yaw_traj = []
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
            self.step_of_traj += 1
            setpoint.position = Point(x=float(x), y=float(y), z=float(z))
            setpoint.velocity = Vector3(x=float(vx), y=float(vy), z=float(vz))
            setpoint.acceleration = Vector3(x=float(ax), y=float(ay), z=float(az))
            setpoint.yaw_rate = 0.0
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

    def generate_trajectory(self, v_max=5.0, a_max=10.0, dt=0.1):
        # vertical takeoff to the first target's height, then A* through every target
        takeoff = [0, 0, self.target_pos[2]]
        targets = [takeoff, self.target_pos]
        waypoints = [self.current_position, takeoff]
        # pairs the targets in couples (T0,T1), (T1,T2) ...
        for p0, p1 in zip(targets[:-1], targets[1:]):
            waypoints += self.plan_path(p0, p1)[1:] # [1:] to avoid repeating the takeoff point twice
        self.get_logger().info(f'A*: {len(waypoints)} waypoints: {[[round(v, 2) for v in p] for p in waypoints]}')

        for i, (p0, p1) in enumerate(zip(waypoints[:-1], waypoints[1:])):
            L = math.dist(p0, p1)
            T = max(1.875 * L / v_max, math.sqrt(5.77 * L / a_max), dt)   # from the limits
            c = [self.quintic(p0[k], 0, 0, p1[k], 0, 0, T) for k in range(3)]
            if math.dist(p0[:2], p1[:2]) > 1e-6:                          # face the direction of the segment (kept on vertical ones)
                self.des_yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])

            n = int(round(T / dt))
            ts = np.linspace(0.1, T, n + 1)        # exact samples, last one = T

            for t in ts:                           # t never exceeds T
                (x, vx, ax), (y, vy, ay), (z, vz, az) = (self.sample(ck, t) for ck in c)
                self.pos_traj.append([x, y, z])
                self.vel_traj.append([vx, vy, vz])
                self.acc_traj.append([ax, ay, az])
                self.yaw_traj.append(self.des_yaw)

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
