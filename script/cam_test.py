#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        # Bridge to convert ROS image to OpenCV
        self.bridge = CvBridge()

        # Subscriber to camera
        self.image_sub = self.create_subscription(
            Image,
            '/fastbot_1/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/fastbot_1/cmd_vel', 10)

        self.get_logger().info('LaneFollower node started')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (320, 240))  # resize for speed

            # Crop to bottom half for lane detection
            roi = frame[120:, :]

            # Convert to grayscale and edge detect
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 50, 150)

            # Hough line detection
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=20,
                                    minLineLength=20, maxLineGap=30)

            width = frame.shape[1]
            offset = 0.0

            if lines is not None:
                x_coords = []
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    x_coords.extend([x1, x2])
                lane_center = np.mean(x_coords)
                offset = (lane_center - width / 2) / (width / 2)  # normalized [-1, 1]


            cv2.imshow("Lane Detection", frame)
            cv2.waitKey(1)
            # Velocity command
            twist = Twist()
            twist.linear.x = 1.8  # Forward speed
            twist.angular.z = -offset  # Steer to center

            # self.cmd_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
