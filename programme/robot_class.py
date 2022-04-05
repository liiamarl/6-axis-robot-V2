from dynamixel_control import *
from robot_maths import *
import math as mt
import numpy as np
import random as rd
import time as t
import RPi.GPIO as GPIO


class joint:

    def step_to_degrees(self):
        #gives the joint angle given the servo encoder position
        if self.ID != 16 or not self.multi_turn:
            return 1.0 * (self.PositionStep - self.Middle) / self.Coef
        else:
            #joint 6 when it is in multi-turn mode
            if self.PositionStep > mt.pow(2, 31):
                pos = self.PositionStep - mt.pow(2, 32)
                return 1.0 * (pos - self.Middle - self.offset) / self.Coef
            return 1.0 * (self.PositionStep - self.Middle - self.offset) / self.Coef

    def set_movement_time(self, MoveTime, AccelTime):
        #writes the time of a movement and the acceleration time (both in ms) in the joint's servo
        if 2 * AccelTime > MoveTime:
            AccelTime = MoveTime / 2
        write_data(self.ID, 112, 4, MoveTime)
        write_data(self.ID, 108, 4, AccelTime)

    def move_joint(self, TargetAngle):
        # writes the goal position in the joint's servo
        Steps = int(self.Coef * TargetAngle) + self.Middle
        if self.ID != 16 or not self.multi_turn:
            # most cases
            if Steps > self.Origin and Steps < self.End:
                write_data(self.ID, 116, 4, Steps)
            else:
                print("error, movement not within limits for joint ", self.ID - 10)
        else:
            #joint 6 if it is in multi-turn mode (when the tool doesn't have wires)
            if mt.fabs(self.current_position - TargetAngle) > 180:
                if TargetAngle < 180:
                    self.offset -= 4096
                else:
                    self.offset += 4096
            Steps += self.offset
            write_data(16, 116, 4, Steps)
        self.current_position = TargetAngle
            
        

    def get_position(self):
        # reads the servo position and converts it in degrees
        self.PositionStep = get_servo_position(self.ID)
        self.PositionDeg = self.step_to_degrees()
        return self.PositionDeg

    def __init__(self, ID):
        self.ID = ID
        Origin = [500, 2800, 2300, 0, 500, -1048575] # minimum position of the servo
        Middle = [5413, 8500, 10000, 2048, 2000, 2048] # servo position that correspond to 0 degrees for the joint
        End = [10000, 18000, 12700, 4095, 3650, 1048575] # maximum position of the servo
        StopPin = [17, 22, 27, 0, 0, 0] # pin number on the raspberry pi on witch the endstop is wired (only joint 1, 2 and 3 have one)
        Coef = [55.316, 109.283, 55.316, 11.378, 11.378, 11.378] # number of encoder steps in one degree of joint movement
        P = [2500, 1800, 3000, 2500, 2000, 2000]
        I = [1000, 800, 700, 700, 700, 800]
        D = [1300, 1200, 1000, 1500, 1000, 1500]
        #PID values were selected based on a few tests, probably a lot of room for improvement there
        write_data(self.ID, 84, 2, P[self.ID - 11])#writing the PID values in the servo
        write_data(self.ID, 82, 2, I[self.ID - 11])
        write_data(self.ID, 80, 2, D[self.ID - 11])
        #assignement of the data listed above for the current joint based on his ID
        self.Origin = Origin[self.ID - 11]
        self.Middle = Middle[self.ID - 11]
        self.End = End[self.ID - 11]
        self.StopPin = StopPin[self.ID - 11]
        self.Coef = Coef[self.ID - 11]
        #setting the endstop pins for the first 3 joints
        if self.ID < 14:
            GPIO.setup(self.StopPin, GPIO.IN)
            self.multi_turn = True
        else:
            self.multi_turn = False
        self.PositionStep = get_servo_position(self.ID)
        self.PositionDeg = self.step_to_degrees()
        self.current_position = 0.0
        self.offset = 0

    def go_home(self):
        #homing function for joints 1, 2 and 3
        write_data(self.ID, 64, 1, 0)
        write_data(self.ID, 20, 4, 0)# reseting the servo's offset
        write_data(self.ID, 64, 1, 1)
        if self.ID > 13:
            #this part is useless, joint 4, 5 and 6 don't need to be homed
            write_data(self.ID, 112, 4, 3000)
            write_data(self.ID, 108, 4, 500)
            move_servo(self.ID, self.Origin)
            sleep(3)
        else:
            self.PositionStep = get_servo_position(self.ID)
            print(self.PositionStep)
            if (self.PositionStep < self.Origin - 100):
                #if the servo position is already below the homing position, we know we need at least 1 turn
                self.offset = 4096  #4096 steps = 1 turn
            home = False
            if self.ID == 12:
                self.set_movement_time(2500, 500)#this one has the most weight to carry so i prefer to do it slower
            else:
                self.set_movement_time(1500, 250)
            while not home:
                move_servo(self.ID, self.Origin - self.offset)
                if self.ID == 2:
                    sleep(2.5)
                else:
                    sleep(1.5)
                for i in range(150):
                    #the end switches have huge relyability issues, this is ugly but it works
                    #there are A LOT of false positive on the switch / GPIO but no false negative
                    #we check 150 times if the switch is pressed
                    #the switches are normally closed
                    if not GPIO.input(self.StopPin):
                        home = True
                        break;
                    sleep(.015)
                if home:
                    # switch has been pressed, we write the offset in the servo
                    print("HOME")
                    write_data(self.ID, 64, 1, 0)
                    write_data(self.ID, 20, 4, self.offset)
                    write_data(self.ID, 64, 1, 1)
                else:
                    #switch has not been pressed, increase the offset and do it again
                    self.offset += 4096
                    print("NOT HOME")




class robot:

    def update_position(self):
        #just reads the current angle of each joint and stores it
        self.current_position = forward_kinematics([self.joints["J1"].get_position(), self.joints["J2"].get_position(), self.joints["J3"].get_position(), self.joints["J4"].get_position(), self.joints["J5"].get_position(), self.joints["J6"].get_position()], self.tool_offset)

    def go_to_joint(self, position, move_time, accel_time):
        #quite easy, just set the movement time and goal position for each joint
        if position["faisability"]:
            self.joints["J1"].set_movement_time(move_time, accel_time)
            self.joints["J2"].set_movement_time(move_time, accel_time)
            self.joints["J3"].set_movement_time(move_time, accel_time)
            self.joints["J4"].set_movement_time(move_time, accel_time)
            self.joints["J5"].set_movement_time(move_time, accel_time)
            self.joints["J6"].set_movement_time(move_time, accel_time)
            self.joints["J1"].move_joint(position["joint_angles"][0])
            self.joints["J2"].move_joint(position["joint_angles"][1])
            self.joints["J3"].move_joint(position["joint_angles"][2])
            self.joints["J4"].move_joint(position["joint_angles"][3])
            self.joints["J5"].move_joint(position["joint_angles"][4])
            self.joints["J6"].move_joint(position["joint_angles"][5])
            sleep(move_time / 1000)
            self.update_position()
            while position_distance(position["tool_position"], self.current_position["tool_position"]) > self.error_margin:
                #checking if we are close enough to the goal position to validate the movement.
                sleep(.01)
                self.update_position()
            self.current_position = position

        else:
            print("ERROR : invalid position given")

    def go_to_linear(self, position, move_time, accel_time):
        #goes to the goal position in a "strait" line, also handles rotations around the TCP
        #setting the number of steps (each step will take 100ms)
        if 3 * accel_time > move_time :
            accel_time = int(move_time / 3)
        n_step = int(move_time / 100)
        n_accel_steps = int(accel_time / 100)
        #setting elements that will be used later
        start = self.current_position["tool_position"]
        end = position["tool_position"]
        translation_vector = [end[0][0] - start[0][0], end[0][1] - start[0][1], end[0][2] - start[0][2]]
        rotation_matrix = get_rot_matrix_2(ZYX_to_matrix(start[1]), ZYX_to_matrix(end[1]))
        rotation_rep = matrix_to_axis_angle(rotation_matrix)
        rotation_axis = [rotation_rep[1], rotation_rep[2], rotation_rep[3]]
        rotation_angle = rotation_rep[0]
        #full step sum is the number of full steps + the number of accel steps weighted by their proportion of a full step
        full_step_sum = 1.0 * n_step - 2 * n_accel_steps
        for i in range(n_accel_steps):
            full_step_sum += 2.0 * (i + 1) / (n_accel_steps + 1)
        #full step coef is the proportion of the movement that a full step makes. full step matrix is the same for the rotation
        full_step_coef = 1.0 / full_step_sum
        full_step_matrix = axis_angle_to_matrix(rotation_axis[0], rotation_axis[1], rotation_axis[2], rotation_angle * full_step_coef)
        #computing the proportion of the movement and the corresponding rotation matrix for each acceleration step
        accel_steps_coefs = list()
        accel_matrix = list()
        for i in range(n_accel_steps):
            coef = full_step_coef * (i + 1) / (n_accel_steps + 1)
            accel_steps_coefs.append(coef)
            accel_matrix.append(axis_angle_to_matrix(rotation_axis[0], rotation_axis[1], rotation_axis[2], rotation_angle * coef))
        #computing all the position in the linear path
        move_list = list()
        if n_accel_steps > 0:
            #first one
            translation = [start[0][0] + accel_steps_coefs[0] * translation_vector[0], start[0][1] + accel_steps_coefs[0] * translation_vector[1], start[0][2] + accel_steps_coefs[0] * translation_vector[2]]
            matrix = np.dot(ZYX_to_matrix(start[1]), accel_matrix[0])
            move_list.append(inverse_kinematics([translation, matrix_to_ZYX(matrix)], self.tool_offset, self.current_position["joint_angles"]))
            for i in range(1, n_accel_steps):
                #acceleration
                translation = [move_list[i - 1]["tool_position"][0][0] + accel_steps_coefs[i] * translation_vector[0], move_list[i - 1]["tool_position"][0][1] + accel_steps_coefs[i] * translation_vector[1], move_list[i - 1]["tool_position"][0][2] + accel_steps_coefs[i] * translation_vector[2]]
                matrix = np.dot(ZYX_to_matrix(move_list[i - 1]["tool_position"][1]), accel_matrix[i])
                move_list.append(inverse_kinematics([translation, matrix_to_ZYX(matrix)], self.tool_offset, move_list[i - 1]["joint_angles"]))

            for i in range(n_accel_steps, n_step - n_accel_steps):
                #constant speed
                translation = [move_list[i - 1]["tool_position"][0][0] + full_step_coef * translation_vector[0], move_list[i - 1]["tool_position"][0][1] + full_step_coef * translation_vector[1], move_list[i - 1]["tool_position"][0][2] + full_step_coef * translation_vector[2]]
                matrix = np.dot(ZYX_to_matrix(move_list[i - 1]["tool_position"][1]), full_step_matrix)
                move_list.append(inverse_kinematics([translation, matrix_to_ZYX(matrix)], self.tool_offset, move_list[i - 1]["joint_angles"]))

            ind = n_accel_steps - 1
            for i in range(n_step - n_accel_steps, n_step - 1):
                #decceleration
                translation = [move_list[i - 1]["tool_position"][0][0] + accel_steps_coefs[ind] * translation_vector[0], move_list[i - 1]["tool_position"][0][1] + accel_steps_coefs[ind] * translation_vector[1], move_list[i - 1]["tool_position"][0][2] + accel_steps_coefs[ind] * translation_vector[2]]
                matrix = np.dot(ZYX_to_matrix(move_list[i - 1]["tool_position"][1]), accel_matrix[ind])
                move_list.append(inverse_kinematics([translation, matrix_to_ZYX(matrix)], self.tool_offset, move_list[i - 1]["joint_angles"]))
                ind -= 1

        else:
            #no acceleration
            translation = [start[0][0] + full_step_coef * translation_vector[0], start[0][1] + full_step_coef * translation_vector[1], start[0][2] + full_step_coef * translation_vector[2]]
            matrix = np.dot(ZYX_to_matrix(start[1]), full_step_matrix)
            move_list.append(inverse_kinematics([translation, matrix_to_ZYX(matrix)], self.tool_offset, self.current_position["joint_angles"]))

            for i in range(1, n_step - 1):
                translation = [move_list[i - 1]["tool_position"][0][0] + full_step_coef * translation_vector[0], move_list[i - 1]["tool_position"][0][1] + full_step_coef * translation_vector[1], move_list[i - 1]["tool_position"][0][2] + full_step_coef * translation_vector[2]]
                matrix = np.dot(ZYX_to_matrix(move_list[i - 1]["tool_position"][1]), full_step_matrix)
                move_list.append(inverse_kinematics([translation, matrix_to_ZYX(matrix)], self.tool_offset, move_list[i - 1]["joint_angles"]))
        #setting the last move with the goal position data to avoid small mistakes that might add up in all the steps
        move_list.append(inverse_kinematics(position["tool_position"], self.tool_offset, move_list[n_step - 2]["joint_angles"]))

        for move in move_list:
            #checking if every position in the path is ok
            if not move["faisability"]:
                print ("error : unreachable position in the linear path")
                print(move["tool_position"])
                print(move["joint_angles"])
                return 0

        accel_delay = 25
        #setting the movement time to 100ms + a small time for acceleration
        #if the accel_time is to small, it will jerk
        #if it is to large, the movement will not be strait
        self.joints["J1"].set_movement_time(100 + 2 * accel_delay, accel_delay)
        self.joints["J2"].set_movement_time(100 + 2 * accel_delay, accel_delay)
        self.joints["J3"].set_movement_time(100 + 2 * accel_delay, accel_delay)
        self.joints["J4"].set_movement_time(100 + 2 * accel_delay, accel_delay)
        self.joints["J5"].set_movement_time(100 + 2 * accel_delay, accel_delay)
        self.joints["J6"].set_movement_time(100 + 2 * accel_delay, accel_delay)

        start_time = time.time()#recording the time

        for i in range(n_step):
            while time.time() < start_time + 0.1 * i:
                #wait for the right moment to start the next step
                #should start about 50ms before the last one is completed
                sleep(.001)
            self.joints["J1"].move_joint(move_list[i]["joint_angles"][0])
            self.joints["J2"].move_joint(move_list[i]["joint_angles"][1])
            self.joints["J3"].move_joint(move_list[i]["joint_angles"][2])
            self.joints["J4"].move_joint(move_list[i]["joint_angles"][3])
            self.joints["J5"].move_joint(move_list[i]["joint_angles"][4])
            self.joints["J6"].move_joint(move_list[i]["joint_angles"][5])
        sleep(.15)
        self.update_position()
        while position_distance(position["tool_position"], self.current_position["tool_position"]) > self.error_margin:
            #checking if we are close enough from the goal position to validate the move
            sleep(.01)
            self.update_position()
        self.current_position = move_list[n_step - 1]


    def rotate_around(self,axis, angle, move_time, accel_time):
        #rotates the robot around X, Y or Z
        #this function shouldn't have been written in the first place since go_to_linear can do the same thing
        #but now it is there, it is used in the code and it works so i'll leave it
        
        #defining the numbers of steps in total and needed for acceleration
        if 3 * accel_time > move_time :
            accel_time = int(move_time / 3)
        n_step = int(move_time / 100)
        n_accel_steps = int(accel_time / 100)

        #getting the proportion of the full rotation each full step makes
        full_step_sum = 1.0 * n_step - 2 * n_accel_steps
        for i in range(n_accel_steps):
            full_step_sum += 2.0 * (i + 1) / (n_accel_steps + 1)
        full_step_coef = 1.0 / full_step_sum
        accel_steps_coefs = list()
        for i in range(n_accel_steps):
            accel_steps_coefs.append(full_step_coef * (i + 1) / (n_accel_steps + 1))


        move_list = list()

        full_step_matrix = get_rot_matrix(angle * full_step_coef, axis)
        
        #basically does the same thing as go_to_linear, but only for rotation around the 3 base axis
        #it works just like go_to_linear


        if n_accel_steps > 0:
            matrix = get_rot_matrix(angle * accel_steps_coefs[0], axis)
            pos = rotate(self.current_position["tool_position"], matrix)
            move_list.append(inverse_kinematics(pos, self.tool_offset, self.current_position["joint_angles"]))

            for i in range(1, n_accel_steps):
                matrix = get_rot_matrix(angle * accel_steps_coefs[i], axis)
                pos = rotate(move_list[i - 1]["tool_position"], matrix)
                move_list.append(inverse_kinematics(pos, self.tool_offset, move_list[i - 1]["joint_angles"]))

            for i in range(n_accel_steps, n_step - n_accel_steps):
                pos = rotate(move_list[i - 1]["tool_position"], full_step_matrix)
                move_list.append(inverse_kinematics(pos, self.tool_offset, move_list[i - 1]["joint_angles"]))

            ind = n_accel_steps - 1
            for i in range(n_step - n_accel_steps, n_step):
                matrix = get_rot_matrix(angle * accel_steps_coefs[ind], axis)
                pos = rotate(move_list[i - 1]["tool_position"], matrix)
                move_list.append(inverse_kinematics(pos, self.tool_offset, move_list[i - 1]["joint_angles"]))
                ind -= 1
        else:
            pos = rotate(self.current_position["tool_position"], full_step_matrix)
            move_list.append(inverse_kinematics(pos, self.tool_offset, self.current_position["joint_angles"]))
            for i in range(1, n_step):
                pos = rotate(move_list[i - 1]["tool_position"], full_step_matrix)
                move_list.append(inverse_kinematics(pos, self.tool_offset, move_list[i - 1]["joint_angles"]))



        for move in move_list:
            if not move["faisability"]:
                print ("error : unreachable position in the linear path")
                print(move["tool_position"])
                print(move["joint_angles"])
                return 0

        accel_delay = 25

        self.joints["J1"].set_movement_time(100 + 2 * accel_delay, accel_delay)
        self.joints["J2"].set_movement_time(100 + 2 * accel_delay, accel_delay)
        self.joints["J3"].set_movement_time(100 + 2 * accel_delay, accel_delay)
        self.joints["J4"].set_movement_time(100 + 2 * accel_delay, accel_delay)
        self.joints["J5"].set_movement_time(100 + 2 * accel_delay, accel_delay)
        self.joints["J6"].set_movement_time(100 + 2 * accel_delay, accel_delay)

        start_time = time.time()

        for i in range(n_step):
            while time.time() < start_time + 0.1 * i:
                sleep(.001)
            self.joints["J1"].move_joint(move_list[i]["joint_angles"][0])
            self.joints["J2"].move_joint(move_list[i]["joint_angles"][1])
            self.joints["J3"].move_joint(move_list[i]["joint_angles"][2])
            self.joints["J4"].move_joint(move_list[i]["joint_angles"][3])
            self.joints["J5"].move_joint(move_list[i]["joint_angles"][4])
            self.joints["J6"].move_joint(move_list[i]["joint_angles"][5])
        sleep(.1)
        self.update_position()
        while position_distance(move_list[n_step - 1]["tool_position"], self.current_position["tool_position"]) > self.error_margin:
            sleep(.01)
            self.update_position()
        self.current_position = move_list[n_step - 1]


    def run_program(self, program):
        #runs a program that was made with the keyboard control
        print("program starts")
        for move in program :
            if len(move["comment"]) > 0:
                print(move["comment"])
            if move["tool_active"] != self.tool_active:
                self.actionate_tool()
            position = inverse_kinematics(move["tool_position"], self.tool_offset, self.current_position["joint_angles"])
            if move["type"] == "j":
                self.go_to_joint(position, move["time"], move["accel"])
                print("ok")
            if move["type"] == "l":
                self.go_to_linear(position, move["time"], move["accel"])
                print("ok")

        print("program ends")

    def set_tool(self, tool_name):
        #when making a new tool, it must be defined here
        if tool_name == "none":
            self.tool = "none"
            self.tool_offset = [0.0, 0.0, 23.0]#setting the tool offset
            self.update_position()#so we must update the tool position
            self.joints["J6"].multi_turn = True#True only if the tool has no wires
        if tool_name == "TCP_demo":
            self.tool = "TCP_demo"
            self.tool_offset = [0.0, 0.0, 60.0]
            self.update_position()
            self.joints["J6"].multi_turn = True

    def actionate_tool(self):
        #when making a new tool, define its action, if any here
        #the tool_active atribut must store the state of the tool
        #if a tool has more than 2 states, the code for that must be changed
        pass



    def __init__(self):
        setup_ports()#opening the communication port with the dynamixels
        GPIO.setwarnings(False)#idk what it does but the internet said i should do it
        GPIO.setmode(GPIO.BCM)
        #setting the default parameters
        self.tool = "none"
        self.tool_offset = [0.0, 0.0, 23.0]
        self.tool_active = False
        self.error_margin = 1
        #creating the joints objects
        self.joints = {"J1" : joint(11), "J2" : joint(12), "J3" : joint(13), "J4" : joint(14), "J5" : joint(15), "J6" : joint(16)}
        enable_torque()
        #homing and getting all joints at the starting position
        #homing in the order J2 -> J3 -> J1 is the safest way
        self.joints["J2"].go_home()
        self.joints["J2"].set_movement_time(3000, 500)
        self.joints["J2"].move_joint(0)
        sleep(3)
        position_J4 = self.joints["J4"].get_position()
        if position_J4 < -90:
            self.joints["J4"].set_movement_time(500, 200)
            self.joints["J4"].move_joint(-90)
            sleep(.5)
        if position_J4 > 90:
            self.joints["J4"].set_movement_time(500, 200)
            self.joints["J4"].move_joint(90)
            sleep(.5)
        self.joints["J3"].go_home()
        self.joints["J3"].set_movement_time(3000, 500)
        self.joints["J3"].move_joint(0)
        sleep(3)
        self.joints["J1"].go_home()
        self.joints["J1"].set_movement_time(2000, 500)
        self.joints["J1"].move_joint(0)
        sleep(2)
        #the first 3 joints now have their home offset
        #we just go to the initial position
        self.joints["J4"].set_movement_time(800, 200)
        self.joints["J4"].move_joint(0)
        sleep(1)
        self.joints["J5"].set_movement_time(800, 200)
        self.joints["J5"].move_joint(0)
        sleep(1)
        self.joints["J6"].set_movement_time(800, 200)
        self.joints["J6"].move_joint(0)
        sleep(1)
        
        self.current_position = forward_kinematics([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], self.tool_offset)
