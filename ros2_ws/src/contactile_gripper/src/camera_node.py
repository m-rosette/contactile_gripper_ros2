#!/usr/bin/env python3

import sys
import os
import rclpy
from std_msgs.msg import String, Float32, Int64, Int32, Float32List
from support import camera
from support.pose_models import CameraGroundTruth

class CameraNode:
    """ROS node for the camera."""
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('camera_node')
        self.cam = camera.Camera()
        self.cam_pub = self.node.create_publisher(Float32List, 'Ground_Truth_Pose', 1)
        self.main_loop_rate = 300
        self.main_loop_rate_obj = self.node.create_rate(self.main_loop_rate)
        self.cam.run()
        self.main_loop()

    def main_loop(self):
        """This is the main loop for the node which executes at self.main_loop_rate."""
        while rclpy.ok():
            self.node.get_logger().debug('In waiting: {}'.format(self.IMU.serial.in_waiting))
            ground_truth = self.cam.find_ground_truth()
            # Publish ground_truth.
            self.main_loop_rate_obj.sleep()

def main():
    camera_node = CameraNode()
    rclpy.spin(camera_node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
