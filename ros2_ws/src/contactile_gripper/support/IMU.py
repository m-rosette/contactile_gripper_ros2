#!/usr/bin/env python3

"""
This module contains a class for the IMU.
"""

import serial
import serial.tools.list_ports
import atexit

import rclpy
from rclpy.node import Node

class IMU(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.dev_board_name = 'Arduino Micro'
        self.baudrate = 115200
        self.com_path = self.find_com_path()
        self.serial = serial.Serial(self.com_path, baudrate=self.baudrate, timeout=1)
        atexit.register(self.shutdown)

    def clean_before_read_start(self):
        """Sets the read protocol pattern to align with the Arduino write protocol.
        This must be run once before starting to continuously read.
        The Arduino write protocol is: 'xdata_ydata_zdata\n'"""
        self.serial.read_until()  # Reads until \n by default.

    def read(self):
        """Returns x, y, z, com_success. com_success is boolean."""
        line = self.serial.readline().decode('ascii').strip('\n\r')
        data = line.split('_')
        # Rarely, the IMU doesn't read all the data. Don't return anything if that happens.
        if len(data) == 3:
            return float(data[0]), float(data[1]), float(data[2]), True
        else:
            return None, None, None, False

    def find_com_path(self):
        """Finds and returns the device port name path as a string. Ex: '/dev/ttyUSB0'"""
        comports = serial.tools.list_ports.comports()
        products = []
        for comport in comports:
            products.append(comport.product)
            if comport.product == self.dev_board_name:
                return comport.device
        self.get_logger().fatal('IMU not found.')
        raise Exception(f'"{self.dev_board_name}" not found in comport products: {products}')

    def shutdown(self):
        self.serial.close()

def main():
    rclpy.init()
    imu = IMU()
    imu.clean_before_read_start()
    while rclpy.ok():
        info = imu.read()
        print(info)
    imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
