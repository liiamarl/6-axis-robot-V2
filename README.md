# 6-axis-robot-V2

This is the second version of my 6 axis robotic arm.

The first version had a lot of mechanical design issues, so i stoped working on it before finishing the software and went to work on this one instead.
It is 3D printed in PETG, uses 6 dynamixels XL330-M288-T servomotors and is controlled by a raspberry Pi, with a U2D2 communication converter between the Pi and the servo bus.


you will find the python code in the "programme" directory.
The full CAD assembly is here in step file.
you can also find the electrical drawings.
There is also a little presentation pdf with pictures of it. 

here is a video of the robot being tested : https://www.reddit.com/r/robotics/comments/tv4i7z/linear_motion_is_done_i_didnt_thought_it_would/

if you want to check how i programmed the robot, the most interresting parts are in robot_maths.py and robot_class.py.
