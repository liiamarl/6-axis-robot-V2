from robot_class import *
import keyboard as kb
from time import sleep
import numpy as np
from program_manager import load_program
from dynamixel_control import *


#this one is executed automatically at boot
#used for demo when there are no screen / keyboard available


setup_ports()

position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(port, 11, 132)
if dxl_comm_result == COMM_SUCCESS:
	R = robot()
	R.set_tool("TCP_demo")
	disable_torque()
	program = load_program("robot/program/demo1.txt")
	while True:
		while GPIO.input(17):
			sleep(0.01)
		enable_torque()
		R.run_program(program)
		disable_torque()
