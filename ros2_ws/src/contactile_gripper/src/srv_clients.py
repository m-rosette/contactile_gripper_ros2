#!/usr/bin/env python3

import rclpy
from contactile_gripper.srv import ChangeMode, DataRecorder, GripperChangeMode, StepperOff, StepperSetLimit, UIMenu
from sensor_interfaces.srv import BiasRequest, StartSlipDetection, StopSlipDetection

def gripper_change_mode_srv_client(mode, node):
    node.get_logger().debug('[gripper_change_mode_srv_client] {}'.format(mode))
    client = node.create_client(GripperChangeMode, 'gripper_change_mode_srv')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn('Service "gripper_change_mode_srv" not available. Waiting...')

    
    request = GripperChangeMode.Request()
    request.mode = mode

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    # node.destroy_node()
    node.get_logger().info(response.response)
    return response.response

# def stepper_off_srv_client(mode):
#     node = rclpy.create_node('stepper_off_node')
#     client = node.create_client(StepperOff, 'stepper_off_srv')
#     while not client.wait_for_service(timeout_sec=1.0):
#         node.get_logger().warn('Service "stepper_off_srv" not available. Waiting...')

#     request = StepperOff.Request()
#     request.mode = mode

#     future = client.call_async(request)
#     rclpy.spin_until_future_complete(node, future)
#     response = future.result()
#     node.destroy_node()
#     return response.response

# def stepper_set_limit_srv_client(switch, action):
#     node = rclpy.create_node('stepper_set_limit_node')
#     client = node.create_client(StepperSetLimit, 'stepper_set_limit_srv')
#     while not client.wait_for_service(timeout_sec=1.0):
#         node.get_logger().warn('Service "stepper_set_limit_srv" not available. Waiting...')

#     request = StepperSetLimit.Request()
#     request.switch = switch
#     request.action = action

#     future = client.call_async(request)
#     rclpy.spin_until_future_complete(node, future)
#     response = future.result()
#     node.destroy_node()
#     return response.response

def data_recorder_srv_client(topic_list, file_prefix='contactile', stop=False):
    node = rclpy.create_node('data_recorder_node')
    client = node.create_client(DataRecorder, 'data_recorder_srv')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn('Service "data_recorder_srv" not available. Waiting...')

    request = DataRecorder.Request()
    request.file_prefix = file_prefix
    request.topic_list = topic_list
    request.stop = stop

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    # node.destroy_node()

def ui_menu_srv_client(menu, node):
    node = rclpy.create_node('ui_menu_node')
    client = node.create_client(UIMenu, 'ui_menu_srv')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn('Service "ui_menu_srv" not available. Waiting...')

    request = UIMenu.Request()
    request.menu = menu

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    # node.destroy_node()

def bias_request_srv_client():
    node = rclpy.create_node('bias_request_node')
    client = node.create_client(BiasRequest, '/hub_0/send_bias_request')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().warn('Service "/hub_0/send_bias_request" not available. Waiting...')

    request = BiasRequest.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    # node.destroy_node()

def main(args=None):
    rclpy.init(args=args)

if __name__ == '__main__':
    main()
