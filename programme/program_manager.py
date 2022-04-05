from os import walk
from time import sleep


#read and write the program as a csv in the program folder




def save_program(program, filepath):
    program_file = open(filepath, 'w')
    for move in program :
        program_file.write(move["type"])
        program_file.write(", ")
        program_file.write(str(move["tool_position"][0][0]))
        program_file.write(", ")
        program_file.write(str(move["tool_position"][0][1]))
        program_file.write(", ")
        program_file.write(str(move["tool_position"][0][2]))
        program_file.write(", ")
        program_file.write(str(move["tool_position"][1][0]))
        program_file.write(", ")
        program_file.write(str(move["tool_position"][1][1]))
        program_file.write(", ")
        program_file.write(str(move["tool_position"][1][2]))
        program_file.write(", ")
        program_file.write(str(move["time"]))
        program_file.write(", ")
        program_file.write(str(move["accel"]))
        program_file.write(", ")
        program_file.write(str(move["tool_active"]))
        program_file.write(", ")
        program_file.write(move["comment"])
        program_file.write("\n")
    program_file.close()



def load_program(filepath):
    program_file = open(filepath, 'r')
    lines = list()
    line = program_file.readline()
    while line != '':
        lines.append(line)
        line = program_file.readline()
    program_file.close()
    program = list()
    for line in lines:
        words = ["", "", "", "", "", "", "", "", "", "", ""]
        ind = 0
        for letter in line :
            if letter == "," and ind < 10:
                ind += 1
            else:
                if (letter != " " or ind == 10) and letter != "\n":
                    words[ind] += letter
            if letter == "\n":
                break
        if ind != 10:
            print("error : program not readable")
            return list()
        move = {"type" : words[0], "tool_position" : [[float(words[1]), float(words[2]), float(words[3])], [float(words[4]), float(words[5]), float(words[6])]], "time" : int(words[7]), "accel" : int(words[8]), "tool_active" : bool(words[9]), "comment" : words[10]}
        program.append(move)
    return program
