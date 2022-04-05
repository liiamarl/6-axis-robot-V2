import RPi.GPIO as IO
from time import sleep
from dynamixel_control import *
from robot_class import *
from keyboard_control import keyboard_loop

#this file is used to make various tests

R = robot()

R.set_tool("TCP_demo")

keyboard_loop(R)



