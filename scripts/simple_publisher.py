#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection 
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from enum import Enum

class RobotControl(Node):
    def __init__(self):
        super().__init__('robotControl')
        self.delta = 0.03
        self.yaw = None
        self.initial_yaw = None
        self.rotation_goal = None
        self.robot_state = RobotState.STARTING
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
        self.timer = self.create_timer(0.01, self.control_robot)

        self.rotating_velocity = Twist()
        self.stop_movement = Twist()
        self.rotating_velocity.angular.z = 0.4
        
    def orientation_goal_reached(self, goal):
        return (goal  + self.delta) % 6.28 > self.yaw and (goal  - self.delta) % 6.28 < self.yaw
    
    def control_robot(self):
        
        if self.robot_state == RobotState.STARTING:

            if self.yaw is None:
                return
            
            if self.initial_yaw is None:
                self.initial_yaw = self.yaw
                self.get_logger().info('started')
            
                self.velocity_publisher.publish(self.rotating_velocity)

            if not self.orientation_goal_reached(self.initial_yaw):
                self.robot_state = RobotState.SEARCH_FOR_MARKERS
                self.rotation_goal = self.initial_yaw

        elif self.robot_state == RobotState.SEARCH_FOR_MARKERS:
            if self.orientation_goal_reached(self.initial_yaw):
                self.velocity_publisher.publish(self.stop_movement)
                self.get_logger().info('stopped')
                self.robot_state = RobotState.MOVE_TO_MARKERS

        elif self.robot_state == RobotState.MOVE_TO_MARKERS:
            print(self.markers_detected)
            exit()
          
        #self.get_logger().info('Publishing: "%s"' % self.velocity.angular)
        
    def detections_callback(self, msg):
        
        if len(msg.markers) > 0:
            for marker in msg.markers:
                if marker.marker_id not in self.markers_detected or \
                    abs(marker.pose.position.x) < abs(self.markers_detected[marker.marker_id][1].x):
                    self.markers_detected[marker.marker_id] = (self.yaw,marker.pose.position)

    def odometry_callback(self, msg):
        
        sin_yaw = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        cos_yaw = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        self.yaw = round(3.14 + math.atan2(sin_yaw, cos_yaw),2)
        #self.get_logger().info(f'yaw: {yaw}')

class RobotState(Enum):
    STARTING  = 1 # starting state
    SEARCH_FOR_MARKERS = 2 # the robot perform a complete turn to find the markers
    MOVE_TO_MARKERS = 3 # the robot rotates to face the marker, goes to it and comes back (backwards)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotControl()
    rclpy.spin(robot_controller)


    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()