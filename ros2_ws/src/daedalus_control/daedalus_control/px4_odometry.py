import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

# NED -> ENU (swap x/y, flip z) and FRD -> FLU (flip y, z); both are their own inverse
NED_TO_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def quat_to_rot(w, x, y, z):
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)]])


def rot_to_quat(R):
    # returns (w, x, y, z), branch on the largest diagonal term for numerical stability
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        return 0.25*s, (R[2, 1] - R[1, 2])/s, (R[0, 2] - R[2, 0])/s, (R[1, 0] - R[0, 1])/s
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return (R[k, j] - R[j, k]) / s, q[0], q[1], q[2]


class Px4Odometry(Node):
    """PX4 EKF odometry (/fmu/out/vehicle_odometry, NED/FRD) -> /odom + TF odom -> base_link (ENU/FLU).

    The odom frame is PX4's local frame turned to ENU, so it is aligned with the Gazebo world
    (x east, y north) and centred on the spawn, like the setpoints of trajectory_generator.
    """

    def __init__(self):
        super().__init__('px4_odometry')
        self.odom_frame = self.declare_parameter('odom_frame_id', 'odom').value
        self.base_frame = self.declare_parameter('base_frame_id', 'base_link').value
        self.publish_tf = self.declare_parameter('publish_tf', True).value

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.update_odometry,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def update_odometry(self, msg):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED or not np.all(np.isfinite(msg.q)):
            return
        # stamp with ROS (sim) time on arrival: same clock as the camera images
        stamp = self.get_clock().now().to_msg()

        p = NED_TO_ENU @ np.asarray(msg.position, dtype=float)
        R = NED_TO_ENU @ quat_to_rot(*msg.q) @ FRD_TO_FLU          # base_link (FLU) -> odom (ENU)
        qw, qx, qy, qz = rot_to_quat(R)

        # twist in base_link, as nav_msgs/Odometry expects
        v = np.asarray(msg.velocity, dtype=float)
        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED:
            v = R.T @ (NED_TO_ENU @ v)
        else:                                                     # already in body FRD
            v = FRD_TO_FLU @ v
        w = FRD_TO_FLU @ np.asarray(msg.angular_velocity, dtype=float)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = p
        o = odom.pose.pose.orientation
        o.w, o.x, o.y, o.z = qw, qx, qy, qz
        t = odom.twist.twist
        t.linear.x, t.linear.y, t.linear.z = np.nan_to_num(v)
        t.angular.x, t.angular.y, t.angular.z = np.nan_to_num(w)

        # covariances from the EKF variances (NED -> ENU swaps x/y); floor so RTAB-Map never
        # gets a zero (= infinitely trusted) covariance
        pv = np.nan_to_num(np.asarray(msg.position_variance, dtype=float), nan=1.0)
        ov = np.nan_to_num(np.asarray(msg.orientation_variance, dtype=float), nan=1.0)
        vv = np.nan_to_num(np.asarray(msg.velocity_variance, dtype=float), nan=1.0)
        pose_var = np.maximum([pv[1], pv[0], pv[2], ov[0], ov[1], ov[2]], [1e-4]*3 + [1e-5]*3)
        twist_var = np.maximum([vv[1], vv[0], vv[2], 1e-3, 1e-3, 1e-3], 1e-4)
        odom.pose.covariance = np.diag(pose_var).flatten().tolist()
        odom.twist.covariance = np.diag(twist_var).flatten().tolist()
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = Px4Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
