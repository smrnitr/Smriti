#!/usr/bin/env python3

################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import cv2
import numpy as np

############################################################

class ArucoDetector(Node):
    def __init__(self):

        self.bridge = CvBridge()
        self.viz_img = np.full((500, 500, 3), (255, 255, 255), dtype=np.uint8)
        self.camera_image = self.viz_img
        self.bot_pose = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_parameters = cv2.aruco.DetectorParameters()

        super().__init__('aruco_detector')

        self.pen_down_subscription_1 = self.create_subscription(
            Bool, "/draw", self.drawingStatusCallback, 10
        )

        self.sub_image = self.create_subscription(
            Image, "/camera/image_raw", self.imageCallback, 10
        )

        self.pose_publisher = self.create_publisher(
            Pose2D, "/bot_pose", 10
        )

    def botPoseCallback(self, msg: Pose2D):
        if self.drawing_status:
            self.bot_pose = (int(msg.x), int(msg.y))

    def drawingStatusCallback(self, msg: Bool):
        self.drawing_status = msg.data

    def imageCallback(self, msg):
        # convert ROS image to opencv image
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
       
        corners, mark, _ = cv2.aruco.detectMarkers(self.camera_image, self.aruco_dict, parameters=self.aruco_parameters)

        self.camera_matrix = np.array([[933.15867, 0, 657.59], [0, 933.1586, 400.36993], [0, 0, 1]])  
        self.dist_coeffs = np.array([-0.43948, 0.18514, 0, 0]) 
        
        if mark is not None:
            for i in range(len(mark)):
                if mark[i] == 1:
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, self.camera_matrix, self.dist_coeffs)
                    x, y, z = tvec[0][0]
                    _, rotation_vector = cv2.Rodrigues(rvec[0])
                    theta = np.arctan2(rotation_vector[1][0], rotation_vector[0][0])

                    pose = Pose2D()
                    pose.x = x
                    pose.y = y
                    pose.theta = theta

                    self.get_logger().info(f"Publishing Pose: x={pose.x}, y={pose.y}, theta={pose.theta}")


                    self.pose_publisher.publish(pose)
                    cv2.aruco.drawDetectedMarkers(self.camera_image, corners)
                    cv2.drawFrameAxes(self.camera_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    break
        else:
            self.get_logger().info("No markers found")

        # Display the image with detected markers
        #cv2.imshow('Aruco Detector', self.camera_image)
        #cv2.waitKey(1)

        
def main(args=None):
    rclpy.init(args=args)
    feedback = ArucoDetector()
    while rclpy.ok():
        # Draw the path
        if feedback.bot_pose:
            feedback.viz_img = cv2.line(
                feedback.viz_img,
                feedback.bot_pose,
                feedback.bot_pose,
                (50, 100, 200),
                3,
            )

        cv2.imshow("Visualization_image", feedback.viz_img)
        cv2.imshow("camera_view", feedback.camera_image)
        cv2.waitKey(1)
        rclpy.spin_once(feedback)
    
    feedback.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
