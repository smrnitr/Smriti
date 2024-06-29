import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from math import sqrt, pow, atan2

class TurtleControllerNode(Node):

    def __init__(self):
        super().__init__("p_controller")
        self.get_logger().info("node p_controller initialized")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        
        self.pose = None

        self.error=0.1
        self.Kp1=1.5
        self.Kp2=6.0
        self.goal_list= [[1.5,8.5],[8.5,8.5],[5.197,5.197],[1.5,1.5],[8.5,1.5],[5.197,5.197]]
        self.goal_variable=0
        self.goal=self.goal_list[self.goal_variable]
        
        
        self.get_logger().info("initialization complete")

    def pose_callback(self, msg):
        self.pose= msg
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta= round(self.pose.theta,4)
        self.get_logger().info(f"updated pose: x=" + str(self.pose.x) + "y=" + str(self.pose.y))
        self.p_controller_move()

            

    def p_controller_move(self):

        if self.pose is None:
            return
        
        
        goal_pose_x, goal_pose_y= self.goal
        distance=sqrt(pow(goal_pose_x-self.pose.x,2)+pow(goal_pose_y-self.pose.y,2))
        if distance < self.error:
            if self.goal_variable <=5:

               self.goal_variable= (self.goal_variable +1)
               self.goal=self.goal_list[self.goal_variable]
               return


        linear_vel=(self.Kp1)*distance
        angle_diff=atan2(goal_pose_y-self.pose.y,goal_pose_x-self.pose.x)
        angular_vel=(self.Kp2)*(angle_diff-self.pose.theta)

        cmd=Twist()
        cmd.linear.x=linear_vel
        cmd.angular.z=angular_vel
              
        self.get_logger().info(f"publishing velocity: x={cmd.linear.x}, z={cmd.angular.z}")
        self.cmd_vel_publisher_.publish(cmd)
           

    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    node.get_logger().info("node initialized in main")
    rclpy.spin(node)   
    rclpy.shutdown()
    node.get_logger().info("node shutdown")


if __name__ == '__main__':
    main()
            
        
    '''
        
        
    def angle_diff(self,point_pose):

        try:
            


            x = float(self.pose.x)
            y = float(self.pose.y)

            self.get_logger().info(f"current pose: x={x}, y={y}")
            self.get_logger().info(f"point pose: x={point_pose.x}, y={point_pose.y}")
            angle_diff=atan2(point_pose.y-self.pose.y,point_pose.x-self.pose.x)
            self.get_logger().info(f"angle difference: {angle_diff}")
            return angle_diff
            

        except Exception as e:
            self.get_logger().error("error in angle difference : {e}")
            return float('inf')
        
    
    def linear_vel(self,point_pose,Kp1=1.8):
        try:
            


            x = float(self.pose.x)
            y = float(self.pose.y)

            self.get_logger().info(f"current pose: x={x}, y={y}")
            self.get_logger().info(f"point pose: x={point_pose.x}, y={point_pose.y}")
            linear_vel=Kp1*self.distance(point_pose)
        
            self.get_logger().info(f"linear velocity: {linear_vel}")
            return linear_vel
            

        except Exception as e:
            self.get_logger().error("error in linear velocity : {e}")
            return float('inf')
        
        
        
    
    def angular_vel(self,point_pose,Kp2=5.8):
        try:
            


            x = float(self.pose.x)
            y = float(self.pose.y)

            self.get_logger().info(f"current pose: x={x}, y={y}")
            self.get_logger().info(f"point pose: x={point_pose.x}, y={point_pose.y}")

            angular_vel=Kp2*(self.angle_diff(point_pose)-self.pose.theta)
            self.get_logger().info(f"angular velocity: {angular_vel}")
            return angular_vel
            
        except Exception as e:
            self.get_logger().error("error in angular velocity : {e}")
            return float('inf')
      
        
    
    def p_controller_move(self,point_pose):
        error=0.1
        cmd=Twist()

        while self.distance(point_pose) >= error:

            linear_vel=(1.5)*self.distance(point_pose)
            angle_diff=atan2(point_pose.y-self.pose.y,point_pose.x-self.pose.x)
            angular_vel=(6.0)*(angle_diff-self.pose.theta)
            cmd.linear.x=linear_vel
            cmd.angular.z=angular_vel
              
            self.get_logger().info(f"publishing velocity: x={cmd.linear.x}, z={cmd.angular.z}")
            self.cmd_vel_publisher_.publish(cmd)
            self.rate.sleep()
            

        cmd.linear.x = 0
        cmd.angular.z = 0
        self.cmd_vel_publisher_.publish(cmd)
        
        
        

    def goal(self,point):

        self.get_logger().info("Entering main method.")
        #points= [[1.5,8.5],[8.5,8.5],[5.197,5.197],[1.5,1.5],[8.5,1.5],[5.197,5.197]]

        
        
        point_pose = Pose()
        point_pose.x = float(point[0])
        point_pose.y = float(point[1])
        print(point_pose.x)
        print(point_pose.y)
            
        self.get_logger().info(f"p_controller algorithm applied for: ({point_pose.x, point_pose.y})")
        self.p_controller_move(point_pose)
            
            
            
        #self.get_logger().info("All points reached")  




def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    node.get_logger().info("node initialized in main")
    try:

       node.main()
    except Exception as e:
        node.get_logger().error("exception in main: {e}")
    finally:

       node.destroy_node()
       node.get_logger().info("node destroyed")
       rclpy.spin(node)
       rclpy.shutdown()
       node.get_logger().info("node shutdown")


if __name__ == '__main__':
    main()
'''