import rclpy
import math
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
import tf2_ros
from daedalus_interfaces.msg import FlightSetpoint


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        self.current_position = None
        self.hold_position = None
        self.last_setpoint_ns = 0
        self.last_mode_request_ns = 0
        self.last_arm_request_ns = 0


        self.setpoint_pos = None
        self.setpoint_vel = None
        self.setpoint_acc = None
        self.setpoint_yaw = None
        self.setpoint_yaw_rate = None
        self.setpoint_count = 0
        
        self.holding_logger = False
        self.offboard_active = False
        self.armed = False

        # SLAM correction: the setpoints are meant in rtabmap's map frame; PX4 flies in its own frame
        # (odom), which drifts without GPS. map -> odom (rtabmap's TF, changed by every loop closure)
        # converts them, applied gradually so a loop closure does not make the setpoint jump
        self.slam_correction = self.declare_parameter('slam_correction', False).value
        self.corr = [0.0, 0.0, 0.0]          # applied correction map -> odom: x, y (ENU, m), yaw (rad)
        self.corr_rate = (0.3, math.radians(5))   # max change: m/s, rad/s
        self.setpoint_raw = None             # last setpoint from the generator (map frame, ENU)
        self.corr_logged = [0.0, 0.0, 0.0]
        if self.slam_correction:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # quality of service def
        px4_output_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        setpoint_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)

        # publishers
        self.control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # subsrcibers
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.update_position,
            px4_output_qos
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.update_status,
            px4_output_qos
        )
        self.create_subscription(
            FlightSetpoint,
            '/drone/setpoint',
            self.update_setpoint,
            setpoint_qos
        )

        # 10Hz loop
        self.create_timer(
            0.1,
            self.offboard_control_loop
        )


    # functions
    def publish_hold(self, timestamp):
        if self.hold_position is None:
            self.hold_position = [float(v) for v in self.current_position]
        control_mode = OffboardControlMode()
        control_mode.timestamp = timestamp
        control_mode.position = True
        self.control_mode_pub.publish(control_mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = self.hold_position
        setpoint.velocity = [math.nan, math.nan, math.nan]
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = math.nan
        setpoint.yawspeed = math.nan
        self.setpoint_pub.publish(setpoint)

    def publish_setpoint(self, timestamp):
        control_mode = OffboardControlMode()
        control_mode.timestamp = timestamp
        control_mode.position = True
        #control_mode.velocity = True
        #control_mode.acceleration = True
        self.control_mode_pub.publish(control_mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = self.setpoint_pos
        setpoint.velocity = self.setpoint_vel
        setpoint.acceleration = self.setpoint_acc
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = self.setpoint_yaw
        setpoint.yawspeed = self.setpoint_yaw_rate
        self.setpoint_pub.publish(setpoint)


    def publish_vehicle_command(self, timestamp, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = timestamp
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    

    # callbacks
    def update_position(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.current_position = (msg.x, msg.y, msg.z)
            
    def update_status(self, msg):
        self.offboard_active = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def update_setpoint(self, msg):
        self.setpoint_raw = msg
        self.apply_setpoint()
        self.last_setpoint_ns = self.get_clock().now().nanoseconds

    def apply_setpoint(self):
        msg = self.setpoint_raw
        p, v, a = msg.position, msg.velocity, msg.acceleration
        # map frame -> PX4's frame (inverse of map -> odom), still ENU
        cx, cy, cyaw = self.corr
        c, s = math.cos(-cyaw), math.sin(-cyaw)
        rot = lambda x, y: (c * x - s * y, s * x + c * y)
        px, py = rot(p.x - cx, p.y - cy)
        vx, vy = rot(v.x, v.y)
        ax, ay = rot(a.x, a.y)
        # ENU (ROS) → NED (PX4): swap x/y, flip z
        self.setpoint_pos = [py, px, -p.z]
        self.setpoint_vel = [vy, vx, -v.z]
        self.setpoint_acc = [ay, ax, -a.z]
        self.setpoint_yaw = math.pi / 2 - (msg.yaw - cyaw)      # NaN stays NaN
        self.setpoint_yaw_rate = -msg.yaw_rate

    def update_correction(self, dt):
        try:
            t = self.tf_buffer.lookup_transform('map', 'odom', Time())
        except tf2_ros.TransformException:
            return                          # no SLAM (yet): no correction
        q = t.transform.rotation
        target = [t.transform.translation.x, t.transform.translation.y,
                  math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))]
        step_xy, step_yaw = self.corr_rate[0] * dt, self.corr_rate[1] * dt
        dx, dy = target[0] - self.corr[0], target[1] - self.corr[1]
        d = math.hypot(dx, dy)
        if d > step_xy:
            dx, dy = dx * step_xy / d, dy * step_xy / d
        dyaw = math.atan2(math.sin(target[2] - self.corr[2]), math.cos(target[2] - self.corr[2]))
        self.corr = [self.corr[0] + dx, self.corr[1] + dy,
                     self.corr[2] + max(-step_yaw, min(step_yaw, dyaw))]
        if (math.hypot(target[0] - self.corr_logged[0], target[1] - self.corr_logged[1]) > 0.2
                or abs(target[2] - self.corr_logged[2]) > math.radians(2)):
            self.corr_logged = target
            self.get_logger().info(
                f'new SLAM correction (loop closure): ({target[0]:.2f}, {target[1]:.2f}) m, '
                f'{math.degrees(target[2]):.1f} deg, applied gradually')


    def offboard_control_loop(self):
        if self.current_position is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        timestamp = now_ns // 1000
        if self.slam_correction:
            self.update_correction(0.1)
            if self.setpoint_raw is not None:
                self.apply_setpoint()

        # 1) ALWAYS stream: generator setpoint if fresh, otherwise hold
        fresh = (self.setpoint_pos is not None
                and now_ns - self.last_setpoint_ns < 500_000_000)
        if fresh:
            self.publish_setpoint(timestamp)
        else:
            self.publish_hold(timestamp)

        # 2) handshake (only after the stream exists)
        self.setpoint_count += 1
        if self.setpoint_count < 10:
            return
        if not self.offboard_active and now_ns - self.last_mode_request_ns > 1_000_000_000:
            self.publish_vehicle_command(timestamp, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.last_mode_request_ns = now_ns
        elif self.offboard_active and not self.armed and now_ns - self.last_arm_request_ns > 1_000_000_000:
            self.publish_vehicle_command(timestamp, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.last_arm_request_ns = now_ns



        


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()