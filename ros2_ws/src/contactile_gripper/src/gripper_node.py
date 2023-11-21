#!/usr/bin/env python3
import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64
from contactile_gripper.srv import *
import support.gripper as gripper


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # Set the logging level for this node
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Initialize and calibrate gripper.
        self.gripper = gripper.Gripper(fast_start=True)

        # Publishers
        self.gripper_pos_pub = self.create_publisher(Int64, 'Gripper_Pos', 1)
        self.gripper_pos = None
        self.gripper_mode_pub = self.create_publisher(String, 'Gripper_Mode', 1)

        self.pub_loop_rate = 60  # Hz
        self.pub_loop_rate_obj = self.create_rate(self.pub_loop_rate)

        # Subscribers
        self.gripper_cmd_sub = self.create_subscription(String, 'Gripper_Cmd', self.gripper_cmd_callback, 1)

        # Services
        self.gripper_change_mode_srv = self.create_service(GripperChangeMode, 'gripper_change_mode_srv', self.srv_handle_change_mode)
        
        # Setup and start
        self.cmd_mode = None
        self.cmd_val = None
        self.change_mode_flag = False
        self.msg_mode_gripper_mode_conversion = {'off': 'passive'}
        # self.on_shutdown(self.shutdown_function)
        self.pub_loop()

    def pub_loop(self):
        while rclpy.ok():
            self.check_if_change_mode()
            self.read_and_publish()
            self.send_command()
            self.pub_loop_rate_obj.sleep()

    def gripper_cmd_callback(self, msg):
        cmd = msg.data.split('_')
        msg_cmd_mode = cmd[0]
        self.cmd_val = int(cmd[1])
        if self.cmd_mode != msg_cmd_mode:
            self.update_cmd_mode(msg_cmd_mode)

    def srv_handle_change_mode(self, request):
        self.get_logger().debug(f'[srv_handle_change_mode] {request.mode}')
        try:
            assert request.mode in self.gripper.mode_options
            if request.mode == 'off':
                self.update_cmd_mode('off')
            elif request.mode == 'cur_based_pos_control':
                self.update_cmd_mode('position')
            elif request.mode == 'cur_control':
                self.update_cmd_mode('current')
        except:
            self.get_logger().error('Change mode service failed.')
        return GripperChangeModeResponse('Mode changed')

    def update_cmd_mode(self, msg_cmd_mode):
        assert msg_cmd_mode == 'position' or msg_cmd_mode == 'current' or msg_cmd_mode == 'off'
        self.change_mode_flag = True
        self.cmd_mode = msg_cmd_mode

    def check_if_change_mode(self):
        if self.change_mode_flag:
            if self.cmd_mode == 'position':
                self.gripper.switch_modes('cur_based_pos_control')
            elif self.cmd_mode == 'current':
                self.gripper.switch_modes('cur_control')
            elif self.cmd_mode == 'off':
                self.gripper.switch_modes('off')
            self.change_mode_flag = False
            self.gripper_mode_pub.publish(String(data=self.gripper.mode))

    def read_and_publish(self):
        if self.gripper.mode != 'off':
            pos, com_err = self.gripper.motor.read_pos()
            if not com_err and pos != 0:
                # self.gripper_pos_pub.publish(pos)
                # self.gripper_pos = pos
                msg = Int64()
                msg.data = pos
                self.gripper_pos_pub.publish(msg)
                self.gripper_pos = pos

    def send_command(self):
        if self.cmd_mode == 'position':
            self.gripper.motor.write_goal_pos(self.cmd_val)
        elif self.cmd_mode == 'current':
            self.gripper.motor.write_goal_cur(self.cmd_val)

    def shutdown_function(self):
        self.gripper.shutdown_function()

def main():
    rclpy.init()
    gripper_node = GripperNode()
    rclpy.spin(gripper_node)
    gripper_node.destroy_node()
    # gripper_node.shutdown_function()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
