#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from contactile_gripper.srv import DataRecorder
import subprocess
import shlex

class DataRecorderNode(Node):

    def __init__(self):
        super().__init__('data_recorder_node')
        self.active_process = None
        self.srv = self.create_service(DataRecorder, 'data_recorder_srv', self.callback)
        self.get_logger().info("\nData recorder node initialized.\n")

    def callback(self, request, response):
        if request.stop:
            self.stop()
            response.success = True
            response.message = "Recording stopped."
        else:
            self.start(request.file_prefix, request.topic_list)
            response.success = True
            response.message = "Recording started."
        return response

    def stop(self):
        self.get_logger().info("\nStop record\n")
        if self.active_process is not None:
            self.active_process.terminate()
            self.active_process = None

    def start(self, file_prefix, topic_list):
        self.get_logger().info("\nStart recording\n")
        msg = f"ros2 bag record -o {file_prefix} " + ' '.join(topic_list)
        args = shlex.split(msg)
        self.active_process = subprocess.Popen(args, stderr=subprocess.PIPE, shell=False)

def main():
    rclpy.init()
    data_recorder_node = DataRecorderNode()
    rclpy.spin(data_recorder_node)
    data_recorder_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
