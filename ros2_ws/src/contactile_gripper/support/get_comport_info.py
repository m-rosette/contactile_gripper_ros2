#!/usr/bin/env python3
"""
Prints info about the currently connected comport devices and paths.
"""

import serial
import serial.tools.list_ports

def print_com_info():
    comports = serial.tools.list_ports.comports()
    for comport in comports:
        info = comport.device, comport.name, comport.description
        print("\n\n#####################################################")
        for key, value in info:
            print("{}: {}".format(key, value))

def get_com_info():
    comport_info = []
    comports = serial.tools.list_ports.comports()
    for comport in comports:
        info = {
            "device": comport.device,
            "name": comport.name,
            "description": comport.description
        }
        comport_info.append(info)
    return comport_info

def main():
    print_com_info()

if __name__ == '__main__':
    main()
