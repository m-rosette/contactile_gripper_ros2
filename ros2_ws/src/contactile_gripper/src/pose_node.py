#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from contactile_gripper.msg import Float32List
from support.pose_models import LinearAnalytical
from sensor_interfaces.msg import SensorState


class PoseNode(Node):
    def __init__(self):
        super().__init__('pose_node', namespace='')

        # Set the logging level for this node
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.pose_model = LinearAnalytical()

        # Publishers
        self.pose_pub = self.create_publisher(Float32List, 'Pose', 1)

        # Subscribers
        self.tact_sensor0 = None
        self.tact_sensor1 = None

        self.tact_sensor0_sub = self.create_subscription(SensorState, '/hub_0/sensor_0', self.tact_0_callback, 1)
        self.tact_sensor1_sub = self.create_subscription(SensorState, '/hub_0/sensor_1', self.tact_1_callback, 1)

        self.main_loop_rate = 30  # Hz
        self.main_loop_rate_obj = self.create_rate(self.main_loop_rate)
        self.wait_for_data()
        self.main_loop()

    def tact_0_callback(self, msg):
        self.tact_sensor0 = msg

    def tact_1_callback(self, msg):
        self.tact_sensor1 = msg

    def main_loop(self):
        while rclpy.ok():
            pose = self.pose_model.predict([self.tact_sensor0, self.tact_sensor1])
            pose_msg = Float32List()
            if pose.position is None:
                # self.pose_pub.publish([float(0), float(0)])
                pose_msg.list = [float(0), float(0)]
            else:
                # self.pose_pub.publish([float(pose.position), float(pose.orientation)])
                pose_msg.list = [float(pose.position), float(pose.orientation)]
            self.pose_pub.publish(pose_msg)
            self.main_loop_rate_obj.sleep()

    def wait_for_data(self):
        while True:
            if self.tact_sensor0 is not None and self.tact_sensor1 is not None:
                return
            self.get_logger().info("PoseNode is waiting for tactile sensor data.")
            rclpy.spin_once(self, timeout_sec=5)

def main(args=None):
    rclpy.init(args=args)
    pose_node = PoseNode()
    # rclpy.spin(pose_node)
    # pose_node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
