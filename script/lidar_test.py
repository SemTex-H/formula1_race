#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/fastbot_1/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('LiDAR subscriber started, listening on /fastbot_1/scan')

    def scan_callback(self, msg: LaserScan):

        n = len(msg.ranges)
        if n == 0:
            return

        self.get_logger().info(f"Total: {n} ")
        center_i = n // 2
        front_angle = msg.angle_min + center_i * msg.angle_increment
        front_range = msg.ranges[center_i]

        self.get_logger().info(
            f"Front beam @ {front_angle:.3f} rad → {front_range:.2f} m"
        )

        closest = min(msg.ranges)
        if closest < 1.0:
            self.get_logger().warn(f"Obstacle at {closest:.2f} m!")

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
