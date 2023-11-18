#!/usr/bin/env python3

import rclpy
from contactile_gripper.srv import ChangeMode, DataRecorder, GripperChangeMode, StepperOff, StepperSetLimit, UIMenu
from std_msgs.msg import String, Int64
from contactile_gripper.msg import Int32List
from support import stepper


class StepperNode:
    def __init__(self):
        self.stepper = stepper()
        rclpy.init()
        self.node = rclpy.create_node('stepper_node', log_level=rclpy.logging.LoggingSeverity.INFO)

        # Publishers
        self.stepper_pos_pub = self.node.create_publisher(Int64, 'Stepper_Pos', 1)
        self.cur_pos = None
        self.limit_switch_pub = self.node.create_publisher(Int32List, 'Limit_Switch_Status', 1)
        self.upper_switch_status = None
        self.lower_switch_status = None

        # Subscribers
        self.stepper_cmd_sub = self.node.create_subscription(String, 'Stepper_Cmd', self.stepper_cmd_callback, 1)

        # Services
        self.stepper_off_srv = self.node.create_service(StepperOff, 'stepper_off_srv', self.srv_handle_stepper_off)
        self.stepper_set_limit_srv = self.node.create_service(StepperSetLimit, 'stepper_set_limit_srv', self.srv_handle_stepper_set_limit)

        self.pub_loop_rate = 60  # Hz
        self.pub_loop_rate_obj = rclpy.create_rate(self.node, self.pub_loop_rate)
        self.upper_lim = None
        self.lower_lim = None
        self.cmd_mode = 'off'
        self.cmd_val = None

        self.node.get_logger().info("Stepper node initialized.")
        self.node.on_shutdown(self.shutdown_function)
        self.pub_loop()

    def srv_handle_stepper_off(self, request, response):
        self.stepper.write('<x_0>')
        self.cmd_mode = 'off'
        response.status_message = 'Stepper off'
        return response

    def srv_handle_stepper_set_limit(self, request, response):
        limit_value = 0
        if request.action == 'clear':
            self.lower_lim = None
            self.upper_lim = None
        elif request.switch == 'upper' and request.action == 'set':
            self.upper_lim = self.cur_pos
            limit_value = self.upper_lim
        elif request.switch == 'lower' and request.action == 'set':
            self.lower_lim = self.cur_pos
            limit_value = self.lower_lim
        else:
            self.node.get_logger().warning('StepperSetLimit service failed. Key error with action: {} or switch: {}'.format(request.action, request.switch))
        response.limit_value = limit_value
        return response

    def stepper_cmd_callback(self, msg):
        cmd = msg.data.split('_')
        self.cmd_mode = cmd[0]
        self.cmd_val = int(cmd[1])

    def pub_loop(self):
        self.stepper.clean_before_read_start()
        while rclpy.ok():
            self.read_data_update_vals()
            self.publish_data()
            self.write_command()
            self.pub_loop_rate_obj.sleep()

    def read_data_update_vals(self):
        lower_switch_status, upper_switch_status, cur_pos, com_success = self.stepper.read()
        self.node.get_logger().info('{} {} {} {}'.format(lower_switch_status, upper_switch_status, cur_pos, com_success))
        if com_success:
            self.cur_pos = int(cur_pos)
            self.upper_switch_status = int(upper_switch_status)
            self.lower_switch_status = int(lower_switch_status)

    def publish_data(self):
        self.stepper_pos_pub.publish(Int64(data=self.cur_pos))
        self.limit_switch_pub.publish(Int32List(data=[self.upper_switch_status, self.lower_switch_status]))

    def write_command(self):
        if self.cmd_mode == 'position':
            self.check_limits_write_pos()
        elif self.cmd_mode == 'speed':
            self.check_limits_write_speed()
        elif self.cmd_mode == 'off':
            pass

    def check_limits_write_pos(self):
        goal_pos = self.cmd_val
        cmd = '<p_' + str(goal_pos) + '>'
        if self.upper_lim is None and self.lower_lim is None:
            self.stepper.write(cmd)
        elif self.upper_lim is not None and self.lower_lim is None:
            if goal_pos < self.upper_lim:
                self.stepper.write(cmd)
        elif self.upper_lim is None and self.lower_lim is not None:
            if goal_pos > self.lower_lim:
                self.stepper.write(cmd)
        elif goal_pos < self.upper_lim and goal_pos > self.lower_lim:
            self.stepper.write(cmd)

    def check_limits_write_speed(self):
        new_speed = self.cmd_val
        cmd = '<s_' + str(new_speed) + '>'
        if self.upper_lim is None and self.lower_lim is None:
            self.stepper.write(cmd)
        elif self.upper_lim is not None and self.lower_lim is None:
            if self.cur_pos < self.upper_lim or new_speed < 0:
                self.stepper.write(cmd)
        elif self.upper_lim is None and self.lower_lim is not None:
            if self.cur_pos > self.lower_lim or new_speed > 0:
                self.stepper.write(cmd)
        # Both an upper and lower limit and cur pos is within limits
        elif self.cur_pos < self.upper_lim and self.cur_pos > self.lower_lim:
            self.stepper.write(cmd)
        # Both an upper and lower limit and cur pos is beyond the upper limit
        elif self.cur_pos > self.upper_lim:
            if new_speed < 0:
                self.stepper.write(cmd)
        # Both an upper and lower limit and cur pos is beyond the lower limit
        elif self.cur_pos < self.lower_lim:
            if new_speed > 0:
                self.stepper.write(cmd)

    def shutdown_function(self):
        self.stepper.serial.close()

def main():
    stepper_node = StepperNode()
    try:
        rclpy.spin(stepper_node)
    except KeyboardInterrupt:
        pass
    finally:
        stepper_node.shutdown_function()
        stepper_node.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
