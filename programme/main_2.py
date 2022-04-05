import RPi.GPIO as IO
from time import sleep
from dynamixel_control import *
from robot_class import *
from keyboard_control import keyboard_loop
from program_manager import load_program

#main file to execute in order to make the robot do stuff
#must be executed as an administrator because of the keyboard module
#be sure that every modules were installed as admin or it wont work

R = robot()

R.set_tool("TCP_demo")

program = load_program("program/demo1.txt")

R.run_program(program)
