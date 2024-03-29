#TODO see if this can easily be moved to the support directory.
# Also, see if the venv at the top level can be deleted.
# Also, see if the /include directory can be deleted. 

import rclpy
from contactile_gripper.srv import ChangeMode, DataRecorder, GripperChangeMode, StepperOff, StepperSetLimit, UIMenu
from sensor_interfaces.srv import BiasRequest, StartSlipDetection, StopSlipDetection

def gripper_change_mode_srv_client(mode):
    rclpy.get_logger().debug('[gripper_change_mode_srv_client] {}'.format(mode))

    gripper_change_mode_srv = rclpy.Node.create_client(GripperChangeMode, 'gripper_change_mode_srv')

    while not gripper_change_mode_srv.wait_for_service(timeout_sec=1.0):
        Node.get_logger().info('service not available, waiting again...')

    msg = gripper_change_mode_srv(mode)
    return msg.response

def stepper_off_srv_client(mode):
    rospy.wait_for_service('stepper_off_srv')
    try:
        srv = rospy.ServiceProxy('stepper_off_srv',StepperOff)
        msg = srv(mode)
        return msg.response
    except:
        rospy.logerr('Change mode service failed.')

def stepper_set_limit_srv_client(switch,action):
    rospy.wait_for_service('stepper_set_limit_srv')
    try:
        srv = rospy.ServiceProxy('stepper_set_limit_srv',StepperSetLimit)
        msg = srv(switch,action)
        return msg.response
    except:
        rospy.logerr('Change mode service failed.')

def data_recorder_srv_client(topic_list,file_prefix='contactile',stop=False):
    rospy.wait_for_service('data_recorder_srv')
    try:
        srv = rospy.ServiceProxy('data_recorder_srv',DataRecorder)
        _ = srv(file_prefix,topic_list,stop)
    except:
        rospy.logerr('data_recorder_srv failed.')

def ui_menu_srv_client(menu):
    rospy.wait_for_service('ui_menu_srv')
    srv = rospy.ServiceProxy('ui_menu_srv', UIMenu)
    _ = srv(menu)

def bias_request_srv_client():
    rospy.wait_for_service('/hub_0/send_bias_request')
    srv = rospy.ServiceProxy('/hub_0/send_bias_request', BiasRequest)
    success = srv()
    return success
