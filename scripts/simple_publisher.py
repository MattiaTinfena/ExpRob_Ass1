#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from aruco_opencv_msgs.msg import ArucoDetection 
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from enum import Enum

class RobotControl(Node):
    def __init__(self):
        super().__init__('robotControl')

        self.delta = 0.03
        self.dist_treshold = 0.25
        self.dist_home_treshold = 0.05
        self.yaw = None
        self.initial_yaw = None
        self.rotation_goal = None
        self.marker_id_goal = None
        self.distance_from_marker = None

        self.robot_state = RobotState.STARTING
        self.markers_detected = dict()
        self.unvisited_markers = []

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
        
        self.timer = self.create_timer(0.007, self.control_robot)

        self.rotating_counterclock_velocity = Twist()
        self.rotating_clock_velocity = Twist()
        self.stop_movement = Twist()
        self.go_to_marker_velocity = Twist()
        self.go_home_velocity = Twist()
        self.robot_position = Point()

        self.rotating_counterclock_velocity.angular.z = 0.3
        self.rotating_clock_velocity.angular.z = -0.3

        self.go_to_marker_velocity.linear.x = 0.3
        self.go_home_velocity.linear.x = -0.3

        
    def orientation_goal_reached(self, goal):
        return (goal  + self.delta) % 6.28 > self.yaw and (goal  - self.delta) % 6.28 < self.yaw
    
    def control_robot(self):
        
        if self.robot_state == RobotState.STARTING:

            if self.yaw is None:
                return
            
            if self.initial_yaw is None:
                self.initial_yaw = self.yaw
                self.get_logger().info('started')
            
                self.velocity_publisher.publish(self.rotating_counterclock_velocity)

            if not self.orientation_goal_reached(self.initial_yaw):
                self.robot_state = RobotState.SEARCH_FOR_MARKERS

        elif self.robot_state == RobotState.SEARCH_FOR_MARKERS:
            if self.orientation_goal_reached(self.initial_yaw):
                self.velocity_publisher.publish(self.stop_movement)
                self.get_logger().info('stopped')

                self.unvisited_markers = list(self.markers_detected.keys())
                self.unvisited_markers.sort(reverse = True)
                self.robot_state = RobotState.SET_MARKER_GOAL

        elif self.robot_state == RobotState.SET_MARKER_GOAL:
            if len(self.unvisited_markers) != 0:
                self.marker_id_goal = self.unvisited_markers.pop()
                # self.marker_id_goal = 256
                self.get_logger().info(f'current marker id goal: {self.marker_id_goal}')
                self.robot_state = RobotState.ROTATE_TO_MARKER
            else:
                self.get_logger().info('visited all markers')
                exit()
        
        elif self.robot_state == RobotState.ROTATE_TO_MARKER:
            self.rotation_goal = self.markers_detected[self.marker_id_goal][0]   

            if not self.orientation_goal_reached(self.rotation_goal):
                if (self.rotation_goal - self.yaw) % 6.28 > 3.14:
                    self.velocity_publisher.publish(self.rotating_clock_velocity) 
                else:
                    self.velocity_publisher.publish(self.rotating_counterclock_velocity)
            else:
                self.get_logger().info('orientation reached')
                self.velocity_publisher.publish(self.stop_movement)
                self.robot_state = RobotState.GO_TO_MARKER
        
        elif self.robot_state == RobotState.GO_TO_MARKER:
            #self.get_logger().info('reaching marker...')
            self.velocity_publisher.publish(self.go_to_marker_velocity)

            if self.distance_from_marker and abs(self.distance_from_marker) < self.dist_treshold :
                self.velocity_publisher.publish(self.stop_movement)
                #TODO: acquire image and publish it
                self.get_logger().info('marker reached')
                self.robot_state = RobotState.COMING_BACK

        elif self.robot_state == RobotState.COMING_BACK:
            self.velocity_publisher.publish(self.go_home_velocity)
            if math.sqrt(self.robot_position.x**2 + self.robot_position.y**2) < self.dist_home_treshold:
                self.velocity_publisher.publish(self.stop_movement)
                self.get_logger().info('home reached')
                self.robot_state = RobotState.SET_MARKER_GOAL
                


        #self.get_logger().info('Publishing: "%s"' % self.velocity.angular)
        
    def detections_callback(self, msg):
        if len(msg.markers) > 0 and self.robot_state == RobotState.SEARCH_FOR_MARKERS:
            for marker in msg.markers:
                if marker.marker_id not in self.markers_detected or \
                    abs(marker.pose.position.x) < abs(self.markers_detected[marker.marker_id][1].x):
                    self.markers_detected[marker.marker_id] = (self.yaw,marker.pose.position)
        
        if self.robot_state == RobotState.GO_TO_MARKER or self.robot_state == RobotState.COMING_BACK:
            for marker in msg.markers:
                if marker.marker_id == self.marker_id_goal:
                    self.distance_from_marker = marker.pose.position.z
                    break

    def odometry_callback(self, msg):        
        sin_yaw = 2 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y)
        cos_yaw = 1 - 2 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z)
        self.yaw = round(3.14 + math.atan2(sin_yaw, cos_yaw),2)
        self.robot_position = msg.pose.pose.position
        #self.get_logger().info(f'yaw: {yaw}')

class RobotState(Enum):
    STARTING  = 1 # starting state
    SEARCH_FOR_MARKERS = 2 # the robot perform a complete turn to find the markers
    # the robot rotates to face the marker, goes to it and comes back (backwards)
    SET_MARKER_GOAL = 3
    ROTATE_TO_MARKER = 4
    GO_TO_MARKER = 5
    COMING_BACK = 6

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotControl()
    rclpy.spin(robot_controller)


    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()