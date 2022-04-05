from robot_class import *
from time import sleep
import keyboard as kb
from os import walk
from program_manager import *
import copy


#the keyboard control loop is defined here, it's mostly boring stuff.


def print_help():
    print("welcome to the keyboard control tool for this robot arm.")
    print("you can move the robot by step, and change the size of the steps")
    print("all moves are made in the world frame")
    print("by default the step size is 10mm and 10 degrees, but you can change it")
    print("this tool also allows you to make and save programs for the robot")
    print("you can also disable the robot's motors and move it by hand to get to a position")
    print("list of commands (press the keys together): ")
    print("    x and + : moves the robot one step forward in the x axis (front)")
    print("    x and - : moves the robot one step backward in the x axis(back)")
    print("    y and + : moves the robot one step forward in the y axis (left)")
    print("    y and - : moves the robot one step backward in the y axis(right)")
    print("    z and + : moves the robot one step forward in the z axis (up)")
    print("    z and - : moves the robot one step backward in the z axis(down)")
    print("    w and + : moves the robot one step around the z axis (counter-clockwise)")
    print("    w and - : moves the robot one step around the z axis (clockwise)")
    print("    p and + : moves the robot one step around the y axis (counter-clockwise)")
    print("    p and - : moves the robot one step around the y axis (clockwise)")
    print("    r and + : moves the robot one step around the x axis (counter-clockwise)")
    print("    r and - : moves the robot one step around the x axis (clockwise)")
    print("    t : activates / disactivates the tool (if there is a active tool)")
    print("    s and + : increase the size of a step")
    print("    s and - : decrease the size of a step")
    print("    d : define the tool (will ask you for its name witch must be defined in the robot class)")
    print("    m : enable / disable the motor's torque")
    print("    p and l: add the current position to the program as a linear move")
    print("    p and j: add the current position to the program as a joint move")
    print("    s and p : save the program (will ask you for a name)")
    print("    l and s : gives a list of the programs currently saved")
    print("    c : clears the current program")
    print("    o : opens a program (will ask you for its name)")
    print("    g : runs the program currently open")
    print("    h : displays this message")
    print("    e : exit")

def clear_inputs():
    #i didn't find any ways to clear the keyboards inputs used for the "is_pressed" from the teminal before an input() call on the internet
    #so we just dump them in a input by pressing enter before every input()calls. it's not ideal but it does the trick.
    print(" ")
    x = input("press enter")
    

def keyboard_loop(R):

    print_help()

    torque_on = True

    moved_since_torque_off = False


    program = list()

    step_size = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15, 20, 25, 30, 40, 45, 50, 60, 70, 80, 90]
    step_time = [200, 200, 300, 400, 400, 500, 500, 600, 700, 800, 900, 1000, 1000, 1200, 1500, 1600, 1800, 2000, 2200, 2400, 2600]
    step_accel = [0, 0, 0, 100, 100, 100, 100, 100, 100, 200, 200, 200, 300, 300, 230, 300, 400, 400, 500, 500, 500]
    step_ind = 9

    exit_loop = False

    while not exit_loop:
        

        if kb.is_pressed("x") and kb.is_pressed("+") :
            tool_position = copy.deepcopy(R.current_position["tool_position"])
            tool_position[0][0] += step_size[step_ind]
            position = inverse_kinematics(tool_position, R.tool_offset, R.current_position["joint_angles"])
            if position["faisability"]:
                R.go_to_linear(position, step_time[step_ind], step_accel[step_ind])
                moved_since_torque_off = True

        if kb.is_pressed("x") and kb.is_pressed("-") :
            tool_position = copy.deepcopy(R.current_position["tool_position"])
            tool_position[0][0] -= step_size[step_ind]
            position = inverse_kinematics(tool_position, R.tool_offset, R.current_position["joint_angles"])
            if position["faisability"]:
                R.go_to_linear(position, step_time[step_ind], step_accel[step_ind])
                moved_since_torque_off = True

        if kb.is_pressed("y") and kb.is_pressed("+") :
            tool_position = copy.deepcopy(R.current_position["tool_position"])
            tool_position[0][1] += step_size[step_ind]
            position = inverse_kinematics(tool_position, R.tool_offset, R.current_position["joint_angles"])
            if position["faisability"]:
                R.go_to_linear(position, step_time[step_ind], step_accel[step_ind])
                moved_since_torque_off = True

        if kb.is_pressed("y") and kb.is_pressed("-") :
            tool_position = copy.deepcopy(R.current_position["tool_position"])
            tool_position[0][1] -= step_size[step_ind]
            position = inverse_kinematics(tool_position, R.tool_offset, R.current_position["joint_angles"])
            if position["faisability"]:
                R.go_to_linear(position, step_time[step_ind], step_accel[step_ind])
                moved_since_torque_off = True

        if kb.is_pressed("z") and kb.is_pressed("+") :
            tool_position = copy.deepcopy(R.current_position["tool_position"])
            tool_position[0][2] += step_size[step_ind]
            position = inverse_kinematics(tool_position, R.tool_offset, R.current_position["joint_angles"])
            if position["faisability"]:
                R.go_to_linear(position, step_time[step_ind], step_accel[step_ind])
                moved_since_torque_off = True

        if kb.is_pressed("z") and kb.is_pressed("-") :
            tool_position = copy.deepcopy(R.current_position["tool_position"])
            tool_position[0][2] -= step_size[step_ind]
            position = inverse_kinematics(tool_position, R.tool_offset, R.current_position["joint_angles"])
            if position["faisability"]:
                R.go_to_linear(position, step_time[step_ind], step_accel[step_ind])
                moved_since_torque_off = True

        if kb.is_pressed("w") and kb.is_pressed("+") :
            R.rotate_around("Z", step_size[step_ind], step_time[step_ind], step_accel[step_ind])
            moved_since_torque_off = True
        if kb.is_pressed("w") and kb.is_pressed("-") :
            R.rotate_around("Z", -step_size[step_ind], step_time[step_ind], step_accel[step_ind])
            moved_since_torque_off = True

        if kb.is_pressed("p") and kb.is_pressed("+") :
            R.rotate_around("Y", step_size[step_ind], step_time[step_ind], step_accel[step_ind])
            moved_since_torque_off = True
        if kb.is_pressed("p") and kb.is_pressed("-") :
            R.rotate_around("Y", -step_size[step_ind], step_time[step_ind], step_accel[step_ind])
            moved_since_torque_off = True

        if kb.is_pressed("r") and kb.is_pressed("+") :
            R.rotate_around("X", step_size[step_ind], step_time[step_ind], step_accel[step_ind])
            moved_since_torque_off = True
        if kb.is_pressed("r") and kb.is_pressed("-") :
            R.rotate_around("X", -step_size[step_ind], step_time[step_ind], step_accel[step_ind])
            moved_since_torque_off = True

        if kb.is_pressed("t") :
            R.actionate_tool()

        if kb.is_pressed("s") and kb.is_pressed("+"):
            if step_ind < 20:
                step_ind += 1
                print ("the size of a step is now ", step_size[step_ind], " mm / degrees")
            else:
                print("maximum reached")
            sleep(.5)

        if kb.is_pressed("s") and kb.is_pressed("-"):
            if step_ind > 0:
                step_ind -= 1
                print ("the size of a step is now ", step_size[step_ind], " mm / degrees")
            else:
                print("minimum reached")
            sleep(.5)

        if kb.is_pressed("d"):
            clear_inputs()
            print("give the tool's name (must be defined in the robot class)")
            tool_name = input()
            R.set_tool(tool_name)
            R.update_position()

        if kb.is_pressed("m"):
            if torque_on:
                disable_torque()
                torque_on = False
                moved_since_torque_off = False
                print("torque disabled")
            else:
                enable_torque()
                torque_on = True
                print("torque enabled")


        if kb.is_pressed("l") and kb.is_pressed("p"):
            clear_inputs()
            move_time = 0
            accel_time = 0
            if len(program) > 0:
                print ("in how much time must this move be made (in milliseconds)")
                move_time = int(input())
                if move_time > 399:
                    accel_time = 100
                if move_time > 799:
                    accel_time = 200
                if move_time > 1199:
                    accel_time = 300
                if move_time > 1499:
                    accel_time = 400
                if move_time > 1999:
                    accel_time = 500
            else:
                move_time = 4000
                accel_time = 500
            print("add a comment for this move:")
            comment = input()
            if not moved_since_torque_off:
                R.update_position()
            move = {"type" : "l", "tool_position" : R.current_position["tool_position"], "time" : move_time, "accel" : accel_time, "tool_active" : R.tool_active, "comment" : comment}
            if len(program) == 0:
                move["type"] = "j"

            program.append(move)
            print("move registred")

        if kb.is_pressed("j") and kb.is_pressed("p"):
            clear_inputs()
            move_time = 0
            accel_time = 0
            if len(program) > 0:
                print ("in how much time must this move be made (in milliseconds)")
                move_time = int(input())
                if move_time > 399:
                    accel_time = 100
                if move_time > 799:
                    accel_time = 200
                if move_time > 1199:
                    accel_time = 300
                if move_time > 1499:
                    accel_time = 400
                if move_time > 1999:
                    accel_time = 500
            else:
                move_time = 4000
                accel_time = 500
            print("add a comment for this move:")
            comment = input()
            if not moved_since_torque_off:
                R.update_position()
            move = {"type" : "j", "tool_position" : R.current_position["tool_position"], "time" : move_time, "accel" : accel_time, "tool_active" : R.tool_active, "comment" : comment}
            program.append(move)
            print("move registred")

        if kb.is_pressed("s") and kb.is_pressed("p"):
            clear_inputs()
            print("write the programs name (if it already exist it will be overwritten)")
            name = input()
            filepath = "program/" + name
            save_program(program, filepath)
            print("program saved")

        if kb.is_pressed("s") and kb.is_pressed("l"):
            clear_inputs()
            program_list = []
            for (dirpath, dirnames, filenames) in walk("program"):
                program_list.extend(filenames)
                break
            if len(program_list) > 0:
                print ("liste des programmes disponibles : ")
            for name in program_list:
                print(name)
                
        if kb.is_pressed("c"):
            program = list()
            print("program cleared")
            sleep(.5)

        if kb.is_pressed("o"):
            clear_inputs()
            print ("give the name of the program you want to load (don't forget the .txt)")
            name = input()
            filepath = "program/" + name
            program = load_program(filepath)
            if len(program) == 0:
                print("error : empty program")
            else:
                print("program loaded")

        if kb.is_pressed("g"):
            if not torque_on:
                enable_torque()
                torque_on = True
            R.run_program(program)

        if kb.is_pressed("h"):
            print_help()
            sleep(.5)

        if kb.is_pressed("e"):
            clear_inputs()
            print ("are you sure you want to exit ? yes / no")
            ans = input()
            if ans == "yes":
                exit_loop = True
        sleep(.05)
    print("out")
