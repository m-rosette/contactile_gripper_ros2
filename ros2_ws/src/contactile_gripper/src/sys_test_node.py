#!/usr/bin/env python3

import rclpy
import sys
from contactile_gripper.msg import Float32List


class TestNode:
    """Test hardware and publish a message if everything looks good."""
    def __init__(self, args):
        rclpy.init(args=args)
        self.node = rclpy.create_node('sys_test_node', namespace='', log_level=rclpy.logging.LoggingSeverity.INFO)
        self.config = args
        if "camera" in self.config:
            pass
        if "IMU" in self.config:
            pass
        self.IMU_pub = self.node.create_publisher(Float32List, 'IMU_Acc', 1)
        self.main_loop_rate = 300  # Hz
        self.main_loop_rate_obj = rclpy.create_rate(self.node, self.main_loop_rate)

        self.IMU = None  # You need to create an instance of your IMU class

        self.IMU.clean_before_read_start()
        self.main_loop()

    def main_loop(self):
        """This is the main loop for the node which executes at self.main_loop_rate."""
        while rclpy.ok():
            self.node.get_logger().debug('In waiting: {}'.format(self.IMU.serial.in_waiting))
            x, y, z, com_success = self.IMU.read()
            if com_success:
                self.IMU_pub.publish(data=[x, y, z])
            self.main_loop_rate_obj.sleep()

def main(args=None):
    rclpy.init(args=args)
    args = sys.argv[1:] if args is None else args
    test_node = TestNode(args)
    try:
        rclpy.spin(test_node.node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
