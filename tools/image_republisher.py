#!/usr/bin/env python3
"""
Republish camera images from several possible source topics (including compressed)
to `/video_source/raw` as `sensor_msgs/msg/Image` so other nodes can subscribe.

Usage:
  source install/setup.bash
  python3 tools/image_republisher.py

It subscribes to common image topic names and republishes the first frames it
receives, converting from compressed if necessary.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2


class ImageRepublisher(Node):
    def __init__(self):
        super().__init__('image_republisher')
        self.bridge = CvBridge()
        qos = 10
        self.pub = self.create_publisher(Image, '/video_source/raw', qos)
        self._frame_counts = {}
        self._last_info_time = self.get_clock().now().seconds_nanoseconds()[0]

        topics = [
            '/marker_publisher/result',
            '/marker_publisher/result/compressed',
            '/video_source/raw',
            '/video_source/raw/compressed',
            '/video_source',
            '/video_source/compressed',
            '/camera/image_raw',
            '/camera/image_raw/compressed',
        ]

        self.get_logger().info(f'Subscribing to candidate topics: {topics}')
        for t in topics:
            try:
                if 'compressed' in t:
                    self.create_subscription(CompressedImage, t, lambda msg, tn=t: self.cb_compressed(msg, tn), qos)
                else:
                    self.create_subscription(Image, t, lambda msg, tn=t: self.cb_image(msg, tn), qos)
            except Exception:
                # ignore missing topics at startup
                pass

    def cb_image(self, msg: Image, topic_name: str):
        try:
            # Forward the Image message directly
            self.pub.publish(msg)
            # Track frames per-topic for lightweight visibility
            self._frame_counts[topic_name] = self._frame_counts.get(topic_name, 0) + 1
            if self._frame_counts[topic_name] % 30 == 0:
                self.get_logger().info(f'Republished Image from {topic_name} -> /video_source/raw (count={self._frame_counts[topic_name]})')
            else:
                self.get_logger().debug(f'Republished Image from {topic_name} -> /video_source/raw')
        except Exception as e:
            self.get_logger().warn(f'Failed to republish image from {topic_name}: {e}')

    def cb_compressed(self, msg: CompressedImage, topic_name: str):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is None:
                self.get_logger().warn(f'Failed to decode compressed image from {topic_name}')
                return
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            self.pub.publish(out_msg)
            self._frame_counts[topic_name] = self._frame_counts.get(topic_name, 0) + 1
            if self._frame_counts[topic_name] % 30 == 0:
                self.get_logger().info(f'Republished CompressedImage from {topic_name} -> /video_source/raw (count={self._frame_counts[topic_name]})')
            else:
                self.get_logger().debug(f'Republished CompressedImage from {topic_name} -> /video_source/raw')
        except Exception as e:
            self.get_logger().warn(f'Failed to handle compressed image from {topic_name}: {e}')


def main():
    rclpy.init()
    node = ImageRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        # ignore double-shutdown errors
        pass


if __name__ == '__main__':
    main()
