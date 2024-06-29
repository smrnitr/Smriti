#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("turtle_controller")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Turtle controller has been started.")
        self.init_point = False
        
        self.clockwise_turtle()
        self.anticlockwise_turtle()
        self.stop_turtle
        
 

    def pose_callback(self, pose: Pose):
        for i in range(1):

           if self.init_point is True:
              self.get_logger().info("Phase stop")
              self.stop_turtle()
           elif pose.x>=5.5444 and pose.y>=5.5444:
              self.get_logger().info("Phase 1")
              self.clockwise_turtle()
           elif pose.x<5.544445 and pose.y>5.549:
              self.get_logger().info("Phase 2")
              self.clockwise_turtle()
           elif pose.x<5.544445 and pose.y<=5.549 and pose.y > 5.44445:
             
              self.get_logger().info("Phase 3")
              self.anticlockwise_turtle()

           elif pose.x>5.544445 and pose.y<5.544445:
            
              self.get_logger().info("Phase 4")
              self.anticlockwise_turtle()
            
           elif pose.x<5.544445 and pose.y<5.544:
            
              self.get_logger().info("Phase 5")
              self.anticlockwise_turtle()
           
        
           

            
        
    


    def anticlockwise_turtle(self):
       cmd = Twist()
       cmd.linear.x = 1.5
       cmd.angular.z = -0.9
       self.cmd_vel_publisher_.publish(cmd)

    

    def clockwise_turtle(self):
       cmd = Twist()
       cmd.linear.x = 1.5
       cmd.angular.z = 0.9
       self.cmd_vel_publisher_.publish(cmd)

    def stop_turtle(self):
       cmd = Twist()
       cmd.linear.x = 0.0
       cmd.angular.z = 0.0
       self.cmd_vel_publisher_.publish(cmd)
       

        


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

'''    def pose_callback(self, pose: Pose):

        
        if not self.init_point:

            if pose.x == 5.544445 and pose.y == 5.544445:

                print('hlw')
                self.init_point = True
                
                self.get_logger().info("Turtle controller has stopped.")
                self.stop_turtle()


            else:

                print('hi')
               
                self.move_turtle()
                self.get_logger().info("Turtle controller has started clockwise.")
                self.init_point = True
                
        else:
            pass
'''