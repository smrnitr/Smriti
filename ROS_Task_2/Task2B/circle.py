import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf_transformations

class OmniRobotCircle(Node):

    def __init__(self, radius):
        super().__init__('omni_robot_circle')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.timer_period = 0.1  # seconds

        self.radius = radius
        self.curr_pose = None
        self.curr_yaw = None

        self.state="ALIGN"

        self.kp1= 0.5  # Proportional gain for linear movement
        self.kp2 = 5.0  # Proportional gain for angular movement

        self.curr_angle = 0.0 # Angle along the circle in radians

    def odom_callback(self, msg):
        self.curr_pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.curr_yaw = OmniRobotCircle.euler_from_quaternion(orientation_q)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            if self.curr_pose is None:
                continue

            # Calculate the target point on the circle
            goal_x = self.radius * (1-math.cos(self.curr_angle+0.01))
            goal_y = self.radius * math.sin(self.curr_angle+0.01)

            distance_error = OmniRobotCircle.calculate_distance(self.curr_pose.x, self.curr_pose.y, goal_x, goal_y)
            goal_yaw = math.atan2(goal_y - self.curr_pose.y, goal_x - self.curr_pose.x)
            yaw_error = OmniRobotCircle.normalize_angle(goal_yaw - self.curr_yaw)
            print(yaw_error)

            msg = Twist()

            if self.state == "ALIGN":

              if abs(yaw_error) > 0.1:  # Adjust angular threshold as needed
                msg.angular.z = self.kp2 * yaw_error
              else:
                
                self.state="MOVE"

            elif self.state=="MOVE":

              msg.linear.x = self.kp1* distance_error
              msg.angular.z = self.kp2* yaw_error

            self.publisher_.publish(msg)

            # Update the current angle to move along the circle
            self.curr_angle += 0.01 # Adjust step size as needed
            if self.curr_angle >= 2 * math.pi:
                self.curr_angle -= 2 * math.pi

    
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
    radius = 2.5 # Example radius; adjust as needed
    omni_robot_circle = OmniRobotCircle(radius)
    omni_robot_circle.run()
    omni_robot_circle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
