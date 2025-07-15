#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarLeaningForward(Node):
    def __init__(self):
        super().__init__('lidar_leaner')

        self.sub = self.create_subscription(
            LaserScan,
            '/fastbot_1/scan',
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(Twist, '/fastbot_1/cmd_vel', 10)
        self.get_logger().info("Leaning-forward LiDAR racer started!")

    def average_range(self, ranges, index, width=5):
        n = len(ranges)
        start = max(index - width, 0)
        end = min(index + width, len(ranges) - 1)
        valid = [r for r in ranges[start:end+1] if 0.05 < r < 10.0]
        return sum(valid) / len(valid) if valid else 0.0

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        left_avg = self.average_range(ranges, 75)
        right_avg = self.average_range(ranges, 285)

        threshold = 1.5
        steer_strength = 0.4
        twist = Twist()

        # Always move forward
        twist.linear.x = 1.8

        # Decision logic
        if left_avg >= threshold and left_avg > right_avg:
            twist.angular.z = steer_strength
            self.get_logger().info(f'LEAN LEFT: L={left_avg:.2f}, R={right_avg:.2f}')
        elif right_avg >= threshold and right_avg > left_avg:
            twist.angular.z = -steer_strength
            self.get_logger().info(f'LEAN RIGHT: L={left_avg:.2f}, R={right_avg:.2f}')
        else:
            twist.angular.z = 0.0
            self.get_logger().info(f'STRAIGHT: L={left_avg:.2f}, R={right_avg:.2f}')

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LidarLeaningForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
