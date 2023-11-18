#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from contactile_gripper.msg import Float32List
from support import IMU

class IMUNode(Node):
    def __init__(self):
        super().__init__('IMU_node', namespace='', log_level=rclpy.logging.LoggingSeverity.INFO)
        self.IMU = IMU.IMU()
        self.IMU_pub = self.create_publisher(Float32List, 'IMU_Acc', 1)
        self.main_loop_rate = 300  # Hz
        self.main_loop_rate_obj = self.create_rate(self.main_loop_rate)
        self.IMU.clean_before_read_start()
        self.main_loop()

    def main_loop(self):
        while rclpy.ok():
            self.get_logger().debug('In waiting: {}'.format(self.IMU.serial.in_waiting))
            x, y, z, com_success = self.IMU.read()
            if com_success:
                self.IMU_pub.publish([x, y, z])
            self.main_loop_rate_obj.sleep()

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
