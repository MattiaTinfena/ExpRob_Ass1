#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection 
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Subscriber(Node):

    def __init__(self):
        super().__init__('topicDetectionSub')
        self.subscription = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'marker id: {msg.markers[0].marker_id}, pose: {msg.markers[0].pose}')


class Publisher(Node):

    def __init__(self):
        super().__init__('speedPub')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.angular.z = 0.0001
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.angular)
        


def main(args=None):
    rclpy.init(args=args)

    topicDetectionSub = Subscriber()
    speedPub = Publisher()

    #rclpy.spin(topicDetectionSub)
    rclpy.spin(speedPub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    topicDetectionSub.destroy_node()
    speedPub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()