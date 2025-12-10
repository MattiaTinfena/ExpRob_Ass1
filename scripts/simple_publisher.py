#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection 
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class RobotControl(Node):
    def __init__(self):
        super().__init__('robotControl')
        # self.timer = self.create_timer(0.1, self.)
        self.delta = 0.03
        self.initial_yaw = None
        self.readyToStop = False
        self.markers_detected = dict()
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detections_subscriber = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.detections_callback,
            10)
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10)
        self.timer = self.create_timer(0.1, self.spin_robot)

        self.velocity = Twist()
        self.velocity.angular.z = 0.7
        
    
    def spin_robot(self):
        self.velocity_publisher.publish(self.velocity)
        #self.get_logger().info('Publishing: "%s"' % self.velocity.angular)
        
    def detections_callback(self, msg):
        if len(msg.markers) > 0:
            #self.get_logger().info(f'marker id: {msg.markers[0].marker_id}, pose: {msg.markers[0].pose}')
            for marker in msg.markers:
                self.markers_detected[marker.marker_id] = marker.pose

    def odometry_callback(self, msg):
        
        sin_yaw = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        cos_yaw = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        yaw = round(3.14 + math.atan2(sin_yaw, cos_yaw),2) 
        #self.get_logger().info(f'yaw: {yaw}')

        if self.initial_yaw is not None:
            near_starting_point = (self.initial_yaw  + self.delta) % 6.28 > yaw and (self.initial_yaw  - self.delta) % 6.28 < yaw
            if not near_starting_point :
                self.readyToStop = True
            if self.readyToStop and near_starting_point:
                #self.get_logger().info(f'reached a complete turn with: {yaw}, initial yaw: {self.initial_yaw}, initial yaw -0.1: {self.initial_yaw - 0.1} stopping...')
                self.readyToStop = False
                self.velocity = Twist()
                #self.get_logger().info(self.markers_detected)
                self.get_logger().info(f'stopped')
        else:
            self.get_logger().info(f'started')
            self.initial_yaw = yaw
            self.spin_robot()


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotControl()
    rclpy.spin(robot_controller)


    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()