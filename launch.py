#!/usr/bin/env python3
"""Launches the gripper. Generates a ROS launch file based on com port findings and config options, and calls roslaunch
with that newly generated file.

The reason that this file is used to launch instead of a typical ROS launch file is because the comport info for the
contactile sensors is not found dynamically on startup. Instead, the comport path is a parameter in the launch file.
This is a problem because the host computer will not reliably give the contactile sensor the same comport. Instead
of finding the comport info every time and editing the launch file, this launch file finds the correct comport and
generates a launch file."""

import serial
import serial.tools.list_ports
import subprocess
import shlex
import os
import time
import logging

logging.basicConfig(level=logging.INFO)

# Config options
stepper = False
IMU = False
camera = False

# File info
launch_dir = "./ros2_ws/src/contactile_gripper/launch/"
gen_file_name = "autogen.launch"
gen_file_path = os.path.join(launch_dir, gen_file_name)
contactile_node_txt = "contactile.txt"
gripper_node_txt = "gripper.txt"
stepper_node_txt = "stepper.txt"
imu_node_txt = "imu.txt"
control_node_txt = "control.txt"
pose_node_txt = "pose.txt"
UI_node_txt = "UI.txt"
sys_test_txt = "sys_test.txt"
data_recorder_path = "./ros2_ws/src/contactile_gripper/src/data_recorder_node.py"

def generate_file():
    """The order matters. Some nodes need to be started before others."""
    with open(gen_file_path, 'w') as genFile:
        genFile.write("\n<launch>")
        write_contactile(genFile)
        write_all_txt(genFile, gripper_node_txt)
        if stepper:
            write_all_txt(genFile, stepper_node_txt)
        if IMU:
            write_all_txt(genFile, imu_node_txt)
        write_all_txt(genFile, pose_node_txt)
        write_all_txt(genFile, control_node_txt)
        write_all_txt(genFile, UI_node_txt)
        # write_sys_test(genFile)
        genFile.write("\n\n</launch>")
        logging.info("Launch file generated.")

def launch():
    # Save the current working directory
    original_dir = os.getcwd()

    # Change the current working directory
    launch_dir = "ros2_ws/src/contactile_gripper/launch/"
    os.chdir(launch_dir) 

    try:
        # Execute the ROS 2 launch command
        command = "ros2 launch {} ".format(gen_file_name)
        args = shlex.split(command)
        process = subprocess.Popen(args, stderr=subprocess.PIPE, shell=False)
    finally:
        # Always change back to the original working directory, even if an exception occurs
        os.chdir(original_dir)

def launch_data_recorder():
    """For some reason, the data recorder node must be started outside of the launch file."""
    time.sleep(5)
    os.system("gnome-terminal -- " + data_recorder_path)

def write_contactile(write_file):
    read_file_path = os.path.join(launch_dir, contactile_node_txt)
    port_path = find_contactile_port()
    with open(read_file_path, 'r') as read_file:
        lines = read_file.readlines()
        write_file.write("\n\n")
        for line in lines:
            if "com_port" in line:
                line = line.replace("TBD", '"' + port_path + '"')
            write_file.write(line)

def write_sys_test(write_file):
    read_file_path = os.path.join(launch_dir, sys_test_txt)
    sys_test_args = find_sys_test_args()
    with open(read_file_path, 'r') as read_file:
        lines = read_file.readlines()
        write_file.write("\n\n")
        for line in lines:
            if "TBD" in line:
                line = line.replace("TBD", '"' + sys_test_args + '"')
            write_file.write(line)

def write_all_txt(write_file, file_txt):
    read_file_path = os.path.join(launch_dir, file_txt)
    with open(read_file_path, 'r') as read_file:
        lines = read_file.readlines()
        write_file.write("\n\n")
        for line in lines:
            write_file.write(line)

def find_contactile_port():
    """Returns the comport path for the contactile sensor hub."""
    comport_info = get_com_info()
    for port in comport_info:
        if port.serial_number == '10129740':
            return port.device
    print("\nContactile sensor hub port info not found.\n")
    raise LookupError

def get_com_info():
    comport_info = list(serial.tools.list_ports.comports())
    return comport_info

def find_sys_test_args():
    args = ""
    if stepper:
        args = args + "-stepper "
    if IMU:
        args = args + "-IMU "
    if camera:
        args = args + "-camera "
    if args == "":
        args = "-none"
    return args

def main():
    generate_file()
    launch()
    # For some reason, the data recorder node must be started outside of the launch file.
    launch_data_recorder()
    logging.info("Launch complete.")

if __name__ == "__main__":
    main()
