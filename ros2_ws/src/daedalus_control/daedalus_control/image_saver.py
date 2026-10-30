import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')

        # parameters
        # base folder: a sub-folder named with the start time is created inside it
        # /ws/images is mounted on the host as <project>/images (docker-compose.yml)
        self.declare_parameter('output_dir', '/ws/images')
        # max saving rate in Hz (0 = save every received frame)
        self.declare_parameter('rate', 2.0)
        # also save the depth images (16-bit PNG, millimetres)
        self.declare_parameter('save_depth', False)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')

        rate = self.get_parameter('rate').value
        self._min_period_ns = int(1e9 / rate) if rate > 0 else 0
        self._bridge = CvBridge()

        # one folder per run, so different flights are not mixed
        run_dir = os.path.join(
            self.get_parameter('output_dir').value,
            datetime.now().strftime('%Y%m%d_%H%M%S'))
        self._color_dir = os.path.join(run_dir, 'color')
        os.makedirs(self._color_dir, exist_ok=True)

        # last saved stamp (ns) and number of saved frames, per stream
        self._last_saved_ns = {'color': None, 'depth': None}
        self._count = {'color': 0, 'depth': 0}

        # subscribers
        # sensor_data qos (best effort) is compatible with both reliable and best effort publishers
        self.create_subscription(
            Image,
            self.get_parameter('color_topic').value,
            self._color_callback,
            qos_profile_sensor_data,
        )
        if self.get_parameter('save_depth').value:
            self._depth_dir = os.path.join(run_dir, 'depth')
            os.makedirs(self._depth_dir, exist_ok=True)
            self.create_subscription(
                Image,
                self.get_parameter('depth_topic').value,
                self._depth_callback,
                qos_profile_sensor_data,
            )

        self.get_logger().info(f'Saving images to {run_dir}')

    def _should_save(self, stream, msg):
        # throttle on the message stamp (sim time), not on wall time
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        last = self._last_saved_ns[stream]
        if last is not None and stamp_ns - last < self._min_period_ns:
            return None
        self._last_saved_ns[stream] = stamp_ns
        return stamp_ns

    def _color_callback(self, msg):
        stamp_ns = self._should_save('color', msg)
        if stamp_ns is None:
            return
        # OpenCV writes BGR
        image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        path = os.path.join(self._color_dir, f'{self._count["color"]:06d}_{stamp_ns}.png')
        cv2.imwrite(path, image)
        self._count['color'] += 1

    def _depth_callback(self, msg):
        stamp_ns = self._should_save('depth', msg)
        if stamp_ns is None:
            return
        # 32FC1 in metres -> uint16 in millimetres (invalid/inf pixels -> 0)
        depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth = np.nan_to_num(depth.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        depth_mm = np.clip(depth * 1000.0, 0, 65535).astype(np.uint16)
        path = os.path.join(self._depth_dir, f'{self._count["depth"]:06d}_{stamp_ns}.png')
        cv2.imwrite(path, depth_mm)
        self._count['depth'] += 1


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Saved {node._count["color"]} colour and {node._count["depth"]} depth images')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
