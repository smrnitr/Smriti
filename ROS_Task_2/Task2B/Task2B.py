#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf_transformations

class Task2BController(Node):

    def __init__(self):
        super().__init__('mini_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 5)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 5)
        self.timer_period = 0.1  # seconds

        # List of coordinate points to form a square
        self.goal_list = [(0, 0), (5, 0), (5, 5), (0, 5), (0, 0)]
        self.goal_index = 0

        self.curr_pose = None
        self.curr_yaw = None

        self.Kp1 = 0.8  # Proportional gain for linear movement
        self.Kp2 = 6.0  # Proportional gain for angular movement

        self.state="ALIGN"

    def odom_callback(self, msg):
        self.curr_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.curr_yaw = Task2BController.euler_from_quaternion(orientation_q)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            if self.curr_pose is None:
                continue

            goal_x, goal_y = self.goal_list[self.goal_index]

            # Calculate the distance and angle to the target point
            distance_error = Task2BController.calculate_distance(self.curr_pose.x, self.curr_pose.y, goal_x, goal_y)
            goal_yaw = math.atan2(goal_y - self.curr_pose.y, goal_x - self.curr_pose.x)
            yaw_error = Task2BController.normalize_angle(goal_yaw - self.curr_yaw)

            msg = Twist()

            if self.state == "ALIGN":
                if abs(yaw_error)>0.1:
                    msg.angular.z = self.Kp2 * yaw_error
                else:
                    self.state="MOVE"

            elif self.state=="MOVE":
                if abs(distance_error)>0.2: # Threshold to consider the point reached
                  msg.linear.x = self.Kp1 * distance_error
                  msg.angular.z = self.Kp2 * yaw_error
                else:
                  self.goal_index += 1
                  if self.goal_index >= len(self.goal_list):
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                    self.publisher_.publish(msg)
                    self.get_logger().info('All points reached. Square path completed.')
                    break
                  self.state="ALIGN"

            self.publisher_.publish(msg)

    
    def calculate_distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

   
    def euler_from_quaternion(quat):
        """
        Convert quaternion to Euler angles.
        """
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        euler = tf_transformations.euler_from_quaternion(quaternion)
        return euler

    
    def normalize_angle(angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    mini_controller = Task2BController()
    mini_controller.run()
    mini_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
