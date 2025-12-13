#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node 
from aruco_opencv_msgs.msg import ArucoDetection 
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from enum import Enum
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class RobotControl(Node):
    def __init__(self):
        super().__init__('robotControl')

        self.bridge = CvBridge()

        self.delta = 0.03
        self.dist_treshold = 0.25
        self.dist_home_treshold = 0.05
        self.yaw = None
        self.initial_yaw = None
        self.rotation_goal = None
        self.marker_id_goal = None
        self.distance_from_marker = None
        self.image_published = False

        self.robot_state = RobotState.STARTING
        self.markers_detected = dict()
        self.unvisited_markers = []

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_publisher = self.create_publisher(CompressedImage, 'camera/image_with_circle', 10)

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
        self.image_subscriber = self.create_subscription(
            CompressedImage, 
            'camera/image/compressed', 
            self.image_callback, 
            10)
        
        cv2.namedWindow("view", cv2.WINDOW_AUTOSIZE)


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
        return ((goal  + self.delta) % (math.pi * 2)) > self.yaw and ((goal  - self.delta) % (math.pi * 2)) < self.yaw
    
    def control_robot(self):
        current_yaw = self.yaw
        
        if self.robot_state == RobotState.STARTING:
            if current_yaw is None:
                return
            
            if self.initial_yaw is None:
                self.initial_yaw = current_yaw
                self.get_logger().info(f'started moving with yaw {current_yaw}')
            
                self.velocity_publisher.publish(self.rotating_counterclock_velocity)

            if not self.orientation_goal_reached(self.initial_yaw):
                self.robot_state = RobotState.SEARCH_FOR_MARKERS

        elif self.robot_state == RobotState.SEARCH_FOR_MARKERS:
            if self.orientation_goal_reached(self.initial_yaw):
                self.velocity_publisher.publish(self.stop_movement)
                self.get_logger().info('stopped')

                self.unvisited_markers = list(self.markers_detected.keys())
                self.unvisited_markers.sort(reverse = True)
                # self.unvisited_markers.pop()
                self.get_logger().info(f'markers detected:{len(self.markers_detected)}')
                for marker_id, marker in self.markers_detected.items():
                    self.get_logger().info(f'marker id:{marker_id}, error: {marker[1]}, yaw: {marker[0]}')
                self.robot_state = RobotState.SET_MARKER_GOAL

        elif self.robot_state == RobotState.SET_MARKER_GOAL:
            if len(self.unvisited_markers) != 0:
                self.marker_id_goal = self.unvisited_markers.pop()
                self.get_logger().info(f'current marker id goal: {self.marker_id_goal}')
                self.robot_state = RobotState.ROTATE_TO_MARKER
            else:
                self.get_logger().info('visited all markers')
                exit()
        
        elif self.robot_state == RobotState.ROTATE_TO_MARKER:
            self.rotation_goal = self.markers_detected[self.marker_id_goal][0]   

            if not self.orientation_goal_reached(self.rotation_goal):
                if (self.rotation_goal - current_yaw) % (math.pi * 2) > math.pi:
                    self.velocity_publisher.publish(self.rotating_clock_velocity) 
                else:
                    self.velocity_publisher.publish(self.rotating_counterclock_velocity)
            else:
                self.get_logger().info(f'orientation reached, with yaw: {current_yaw}')

                self.velocity_publisher.publish(self.stop_movement)
                self.robot_state = RobotState.GO_TO_MARKER
        
        elif self.robot_state == RobotState.GO_TO_MARKER:
            #self.get_logger().info('reaching marker...')
            self.velocity_publisher.publish(self.go_to_marker_velocity)

            if self.distance_from_marker and abs(self.distance_from_marker) < self.dist_treshold :
                self.velocity_publisher.publish(self.stop_movement)
                self.get_logger().info('marker reached')
                self.robot_state = RobotState.ACQUIRE_IMAGE

        elif self.robot_state == RobotState.ACQUIRE_IMAGE:
            if self.image_published:
                self.get_logger().info('image published')
                self.image_published = False  
                self.robot_state = RobotState.COMING_BACK              
            else:
                pass

        elif self.robot_state == RobotState.COMING_BACK:
            self.velocity_publisher.publish(self.go_home_velocity)
            if math.sqrt(self.robot_position.x**2 + self.robot_position.y**2) < self.dist_home_treshold:
                self.velocity_publisher.publish(self.stop_movement)
                self.get_logger().info('home reached')
                self.robot_state = RobotState.SET_MARKER_GOAL
                

        
        #self.get_logger().info('Publishing: "%s"' % self.velocity.angular)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        current_yaw = self.yaw

        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.robot_state == RobotState.SEARCH_FOR_MARKERS or self.robot_state == RobotState.ACQUIRE_IMAGE:
        # if True:
            img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            corners, ids, _ = cv2.aruco.detectMarkers(img_gray, aruco_dict)
            if ids is not None:            
                for i in range(len(ids)):
                    id = ids[i][0]
                    x_center = 0
                    y_center = 0
                    if len(corners[i][0]) != 4:
                        self.get_logger().error(f"numbers of corners wrong!")
                        exit()
                    for corner in corners[i][0]:
                        x_center += corner[0]
                        y_center += corner[1]
                    x_center /= 4.0
                    y_center /= 4.0

                    if self.robot_state == RobotState.ACQUIRE_IMAGE and not self.image_published:
                        
                        # Draw a red circle in the center
                        radius = 0
                        for corner in corners[i][0]:
                            act_dist = math.dist(corner,[x_center,y_center])
                            if radius < act_dist:
                                radius = act_dist

                        cv2.circle(cv_image, (int(x_center), int(y_center)), int(radius), (0, 0, 255), 3)
                        
                        cv2.imshow("view", cv_image)
                        cv2.waitKey(1)    
                        out_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image, dst_format='jpeg')
                        out_msg.header = msg.header  # preserve original header

                        self.image_publisher.publish(out_msg)


                        # Convert back to ROS Image and publish
                        self.image_published = True

                    cv2.circle(cv_image, (int(x_center),240), 6, (255, 0, 0), -1)  # -1 = pieno
                    x_error = abs(640/2 - x_center)
                    if id not in self.markers_detected or \
                        x_error < self.markers_detected[id][1]:
                        if id in self.markers_detected: 
                            self.get_logger().info(f'updating yaw for maker {id}, old error: {self.markers_detected[id][1]} \
                                                old yaw: {self.markers_detected[id][0]} new error: {x_error}, new yaw: {current_yaw}')
                        self.markers_detected[id] = (current_yaw, x_error)
            if ids is not None and len(corners) > 0:
                    for i in range(len(ids)):
                        pts = corners[i][0]  # (4,2): [top-left, top-right, bottom-right, bottom-left] di solito

                        for j in range(4):
                            x, y = pts[j]
                            p = (int(x), int(y))
                            cv2.circle(cv_image, p, 6, (0, 255, 0), -1)  # -1 = pieno

                        cv2.circle(cv_image, (320,240), 6, (0, 255, 0), -1)  # -1 = pieno

                        cv2.putText(cv_image, f"error={x_error}, yaw={current_yaw}", (20, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        
                        cv2.putText(cv_image, f"finalerror={self.markers_detected[id][1]}, finalyaw={self.markers_detected[id][0]}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.putText(cv_image, f"a", (1, 1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            cv2.imshow("view", cv_image)
            cv2.waitKey(1)    
            out_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image, dst_format='jpeg')
            out_msg.header = msg.header  # preserve original header

            self.image_publisher.publish(out_msg)

    
        


    
    def detections_callback(self, msg):
        if self.robot_state == RobotState.GO_TO_MARKER or self.robot_state == RobotState.COMING_BACK:
            for marker in msg.markers:
                if marker.marker_id == self.marker_id_goal:
                    self.distance_from_marker = marker.pose.position.z
                    break

    def odometry_callback(self, msg):
        q = msg.pose.pose.orientation        
        sin_yaw = 2 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw =  math.pi + math.atan2(sin_yaw, cos_yaw)
        self.robot_position = msg.pose.pose.position

class RobotState(Enum):
    STARTING  = 1 # starting state
    SEARCH_FOR_MARKERS = 2 # the robot perform a complete turn to find the markers
    # the robot rotates to face the marker, goes to it and comes back (backwards)
    SET_MARKER_GOAL = 3
    ROTATE_TO_MARKER = 4
    GO_TO_MARKER = 5
    ACQUIRE_IMAGE = 6
    COMING_BACK = 7

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotControl()
    rclpy.spin(robot_controller)


    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()