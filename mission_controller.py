#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import time
from enum import Enum


class State(Enum):
    EXPLORE = 1
    INVESTIGATE = 2
    REPORT = 3
    DONE = 4


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.victim_sub = self.create_subscription(String, '/victim_found', self.victim_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.state = State.EXPLORE
        self.latest_scan = None
        self.pending_victim = None
        self.victims_found = []
        self.mission_start = time.time()
        self.mission_duration_limit = 300.0

        self.timer = self.create_timer(0.2, self.control_loop)
        self.state_enter_time = time.time()

        self.get_logger().info('Mission Controller started. State: EXPLORE')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def victim_callback(self, msg):
        if self.state == State.EXPLORE:
            data = json.loads(msg.data)
            self.pending_victim = data
            self.state = State.INVESTIGATE
            self.state_enter_time = time.time()
            self.get_logger().info('State -> INVESTIGATE (victim signal received)')

    def control_loop(self):
        if self.state == State.EXPLORE:
            self.do_explore()
        elif self.state == State.INVESTIGATE:
            self.do_investigate()
        elif self.state == State.REPORT:
            self.do_report()
        elif self.state == State.DONE:
            self.stop_robot()

        if self.state != State.DONE and (time.time() - self.mission_start) > self.mission_duration_limit:
            self.get_logger().info('Mission time limit reached -> DONE')
            self.state = State.DONE

    def do_explore(self):
        if self.latest_scan is None:
            return
        ranges = [r for r in self.latest_scan.ranges if r > 0.05]
        if not ranges:
            return

        front = min(self.latest_scan.ranges[0:15] + self.latest_scan.ranges[-15:], default=10.0)
        cmd = Twist()
        if front < 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
        else:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def do_investigate(self):
        self.stop_robot()
        if time.time() - self.state_enter_time > 2.0:
            self.state = State.REPORT
            self.state_enter_time = time.time()
            self.get_logger().info('State -> REPORT')

    def do_report(self):
        if self.pending_victim is not None:
            record = dict(self.pending_victim)
            record['report_time'] = time.time()
            self.victims_found.append(record)
            self.get_logger().info(f'LOGGED VICTIM #{len(self.victims_found)}: {record}')
            self.pending_victim = None
        self.state = State.EXPLORE
        self.state_enter_time = time.time()
        self.get_logger().info('State -> EXPLORE')

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.get_logger().info(f'Mission ended. Total victims found: {len(node.victims_found)}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
