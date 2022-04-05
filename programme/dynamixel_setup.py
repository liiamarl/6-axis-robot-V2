from dynamixel_control import *
from robot_class import *


# this is just used to mess around with the dynamixels
# setting up their ID, their drive mode, getting their min / max positions ...


"""
#resets the offset and write the servo position while it is moved manually
setup_ports()
write_data(12, 20, 4, 0)
while True:
    print(get_servo_position(12))
    sleep(.1)
"""


"""
#was used to get the angles at witch joints 3 and 4 make a collision
R = robot()

disable_torque()
while True:
    R.update_position()
    print("J3 :", R.current_position["joint_angles"][2])
    print("J4 :", R.current_position["joint_angles"][3])
    print()
    sleep(.5)


print("ok")
"""

