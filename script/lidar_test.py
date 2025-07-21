#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarLeaner(Node):
    def __init__(self):
        super().__init__('lidar_leaner')

        self.sub = self.create_subscription(
            LaserScan,
            '/fastbot_1/scan',
            self.scan_callback,
            10
        )
        self.pub = self.create_publisher(Twist, '/fastbot_1/cmd_vel', 10)

        self.get_logger().info("Lidar leaner with 1.5m clearance started!")

    def average_range(self, ranges, index, width=5):
        n = len(ranges)
        start = max(index - width, 0)
        end = min(index + width, len(ranges) - 1)
        valid = [r if 0.05 < r < 20.0 else 21.0 for r in ranges[start:end+1]]
        return sum(valid) / len(valid) if valid else 20.0

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        left = self.average_range(ranges, 75)
        right = self.average_range(ranges, 25)

        twist = Twist()
        twist.linear.x = 0.5
        speed_a = 0.3
        amplifier = 0.5
        target_distance = 2
        idx = 0
        
        if left < 10:
            if left > target_distance:
                twist.angular.z = speed_a
                idx = 1
            else:
                twist.angular.z = -speed_a
                idx = 2
        else:
            if right > target_distance:
                twist.angular.z = -speed_a
                idx = 3
            else:
                twist.angular.z = speed_a
                idx = 4

        self.get_logger().info(f"L: {left:.2f}, R: {right:.2f} C: {idx}")

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LidarLeaner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
