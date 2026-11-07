import math
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from daedalus_slam.px4_odometry import frd_to_flu, quat_to_rot, rot_to_quat

# provisional alignment on the ground: PX4's heading must stay within HEADING_SPREAD for HEADING_WINDOW
HEADING_WINDOW = 2.0      # s
HEADING_SPREAD = 1.0      # deg
HEADING_TIMEOUT = 15.0    # s, then align anyway
# final alignment: after ALIGN_DELAY above ALIGN_HEIGHT, when PX4's heading has converged in flight
ALIGN_HEIGHT = 1.5        # m above the start
ALIGN_DELAY = 8.0         # s
# minimum position std sent to PX4, for the whole flight: the positions only have to bound PX4's
# position uncertainty (it refuses to arm / fails safe above COM_POS_FS_EPH = 5 m). Sent with their
# real std (~cm) they override the magnetometer: PX4's heading followed the visual heading drift
# (+3 deg in 20 s), the drone drifted 1.4 m sideways and hit a shelf. Velocity does the short-term work
POS_STD = 2.0             # m
# velocity from positions over VEL_WINDOW: rgbd_odometry's twist is the frame-to-frame motion / 66 ms,
# so a few cm of depth noise per frame become +-0.5 m/s (and tens of m/s after a tracking loss)
VEL_WINDOW = 0.3          # s
VEL_STD = 0.25            # m/s, reported to EKF2 (~noise of the windowed difference)
VEL_MAX = 3.0             # m/s, above this the sample is a glitch (trajectory max is 1 m/s)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


class VisionToPx4(Node):
    """Visual odometry (/odom, odom -> base_link in FLU) -> PX4 external vision, the inverse of px4_odometry.

    rgbd_odometry's odom frame is base_link's start pose, with an arbitrary heading: it is rotated to
    PX4's north and sent as POSE_FRAME_NED, position + body velocity (EKF2_EV_CTRL=5, start.sh --no-gps).
    PX4's heading is ~10 deg wrong on the ground and converges in flight, so the rotation is set twice:
      - provisional, on the ground;
      - final, ALIGN_DELAY after takeoff, keeping the current position (no jump).
    Positions are always sent with a large std (POS_STD), so they bound PX4's position uncertainty
    without overriding its magnetometer heading.
    POSE_FRAME_FRD (EKF2 re-aligning the frame continuously from the visual vs magnetic heading) is not
    used: it rotated every position about the start, tens of metres of error 30 m away.
    """

    def __init__(self):
        super().__init__('vision_to_px4')
        self.pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.create_subscription(Odometry, '/odom', self.forward, 10)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.update_px4,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.px4_heading = None
        self.headings = deque()   # (time [s], heading [rad]) on the ground
        self.start = None
        self.airborne_since = None
        self.align = None         # rotation odom (FRD) -> NED
        self.offset = np.zeros(3)  # translation added after the rotation
        self.final = False
        self.last_ned = None      # last position sent, kept across the final alignment
        self.history = deque()    # (stamp [s], position NED) for the velocity
        self.lost = False

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def update_px4(self, msg):
        if not math.isfinite(msg.heading):
            return
        self.px4_heading = msg.heading
        now = self.now()
        if self.align is None:
            self.headings.append((now, msg.heading))
            while now - self.headings[0][0] > HEADING_WINDOW:
                self.headings.popleft()
        if msg.z_valid and msg.z < -ALIGN_HEIGHT:
            if self.airborne_since is None:
                self.airborne_since = now
        else:
            self.airborne_since = None

    def stable_heading(self):
        now = self.now()
        if self.start is None:
            self.start = now
        if not self.headings or now - self.headings[0][0] < 0.9 * HEADING_WINDOW:
            return None
        h = np.unwrap([a for _, a in self.headings])
        if np.ptp(h) > math.radians(HEADING_SPREAD) and now - self.start < HEADING_TIMEOUT:
            return None
        if np.ptp(h) > math.radians(HEADING_SPREAD):
            self.get_logger().warn('PX4 heading not stable, aligning anyway')
        return float(np.mean(h))

    def update_alignment(self, R_vo, p_vo):
        if self.align is None:
            heading = self.stable_heading()
            if heading is None:
                return False
            self.align = rot_z(heading - yaw_of(R_vo))
            self.get_logger().info(
                f'provisional alignment on the ground: {math.degrees(heading - yaw_of(R_vo)):.1f} deg')
        elif (not self.final and self.airborne_since is not None and
              self.now() - self.airborne_since > ALIGN_DELAY and self.last_ned is not None):
            new = rot_z(self.px4_heading - yaw_of(R_vo))
            change = math.degrees(yaw_of(new) - yaw_of(self.align))
            self.align = new
            # keep the current position: only the motion from now on is rotated differently
            self.offset = self.last_ned - new @ p_vo
            self.offset[2] = 0.0
            self.history.clear()
            self.final = True
            self.get_logger().info(
                f'final alignment in flight: {math.degrees(yaw_of(new)):.1f} deg '
                f'({change:+.1f} deg from the ground)')
        return True

    def velocity(self, stamp, p_ned, R):
        # body-frame velocity from the position change over ~VEL_WINDOW; both in the vision frame,
        # so it does not depend on the visual heading drift (PX4 rotates it with its own heading)
        self.history.append((stamp, p_ned))
        while stamp - self.history[0][0] > VEL_WINDOW:
            self.history.popleft()
        t0, p0 = self.history[0]
        if stamp - t0 < 0.5 * VEL_WINDOW:
            return None
        v = (p_ned - p0) / (stamp - t0)
        if np.linalg.norm(v) > VEL_MAX:
            self.history.clear()
            return None
        return R.T @ v

    def forward(self, odom):
        pose_var = np.asarray(odom.pose.covariance).reshape(6, 6).diagonal()
        # rgbd_odometry publishes a null odometry (zero covariance) while it has lost track
        if not pose_var.any() or pose_var.max() >= 9999:
            if not self.lost:
                self.get_logger().warn('visual odometry lost, nothing sent to PX4')
            self.lost = True
            self.history.clear()   # no velocity across the gap
            return
        if self.lost:
            # rgbd_odometry restarts from its latest pose (Odom/ResetCountdown), so the pose stays
            # continuous: reset_counter is left alone, bumping it makes EKF2 reset its position
            self.lost = False
            self.get_logger().info('visual odometry back')

        p = odom.pose.pose.position
        o = odom.pose.pose.orientation
        R_vo = frd_to_flu @ quat_to_rot(o.w, o.x, o.y, o.z) @ frd_to_flu   # base_link (FRD) -> odom (FRD)
        p_vo = frd_to_flu @ [p.x, p.y, p.z]
        if not self.update_alignment(R_vo, p_vo):
            return
        R = self.align @ R_vo
        p_ned = self.align @ p_vo + self.offset
        self.last_ned = p_ned
        stamp = Time.from_msg(odom.header.stamp).nanoseconds * 1e-9
        v = self.velocity(stamp, p_ned, R)
        w = odom.twist.twist.angular

        msg = VehicleOdometry()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.timestamp_sample = int(stamp * 1e6)
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = p_ned.tolist()
        msg.q = [float(c) for c in rot_to_quat(R)]
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        # NaN = no velocity in this sample (start, after a gap or a glitch); EKF2 skips it
        msg.velocity = [math.nan] * 3 if v is None else v.tolist()
        msg.angular_velocity = (frd_to_flu @ [w.x, w.y, w.z]).tolist()
        msg.position_variance = np.maximum(pose_var[:3], POS_STD ** 2).tolist()
        msg.orientation_variance = pose_var[3:].tolist()
        msg.velocity_variance = [VEL_STD ** 2] * 3
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionToPx4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
