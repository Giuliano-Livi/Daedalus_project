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
        p, v, a = msg.position, msg.velocity, msg.acceleration
        # ENU (ROS) → NED (PX4): swap x/y, flip z
        self.setpoint_pos = [p.y, p.x, -p.z]
        self.setpoint_vel = [v.y, v.x, -v.z]
        self.setpoint_acc = [a.y, a.x, -a.z]
        self.setpoint_yaw = math.pi / 2 - msg.yaw      # NaN stays NaN
        self.setpoint_yaw_rate = -msg.yaw_rate
        self.last_setpoint_ns = self.get_clock().now().nanoseconds


    def offboard_control_loop(self):
        if self.current_position is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        timestamp = now_ns // 1000

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