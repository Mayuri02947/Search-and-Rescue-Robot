#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time


class VictimDetector(Node):
    def __init__(self):
        super().__init__('victim_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(String, '/victim_found', 10)
        self.last_report_time = 0.0
        self.report_cooldown = 5.0
        self.get_logger().info('Victim Detector started — watching for red markers.')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area > 500:
            now = time.time()
            if now - self.last_report_time < self.report_cooldown:
                return
            self.last_report_time = now

            M = cv2.moments(largest)
            cx = int(M['m10'] / M['m00']) if M['m00'] != 0 else frame.shape[1] // 2
            frame_width = frame.shape[1]
            bearing = (cx - frame_width / 2) / (frame_width / 2)

            report = {
                'event': 'victim_detected',
                'timestamp': now,
                'pixel_area': float(area),
                'bearing': round(float(bearing), 3)
            }
            msg_out = String()
            msg_out.data = json.dumps(report)
            self.pub.publish(msg_out)
            self.get_logger().info(f'VICTIM DETECTED — area={area:.0f}, bearing={bearing:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = VictimDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
