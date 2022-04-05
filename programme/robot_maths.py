import numpy as np
import math as mt
import random as rd

# for the conversion formulas between matrix and angles see https://en.wikipedia.org/wiki/Euler_angles

def matrix_to_ZYZ(matrix):
    # converts a rotation matrix to ZYZ proper euler angles witch correspond to the robot's wrist joints
    # it returns only one of the two solutions, the other one must be taken into consideration outside of this function
    ZYZ_angles = [0.0, 0.0, 0.0]
    if mt.fabs(matrix[0][2]) < 0.0000001:
        if matrix[1][2] < -0.00000001:
            ZYZ_angles[0] = -90
        else:
            if matrix[1][2] > 0.00000001:
                ZYZ_angles[0] = 90
            else:
                ZYZ_angles[0] = 0
    else:
        ZYZ_angles[0] = mt.degrees(mt.atan2(matrix[1][2], matrix[0][2]))

    if mt.fabs(matrix[2][2]) < 0.0000001:
        ZYZ_angles[1] = 90
    else:
        ZYZ_angles[1] = mt.degrees(mt.atan2(mt.sqrt(1 - mt.pow(matrix[2][2], 2)), matrix[2][2]))
    if mt.fabs(ZYZ_angles[1] < .0001):
        #sigularity, will be dealt with outside of this function
        ZYZ_angles[0] = mt.degrees(mt.acos(matrix[0][0]))
        return ZYZ_angles
    if mt.fabs(ZYZ_angles[1] - 180) < 0.000001:
        #sigularity, will be dealt with outside of this function
        ZYZ_angles[0] = mt.degrees(mt.acos(matrix[1][1]))
        return ZYZ_angles

    if mt.fabs(matrix[2][0]) < 0.0000001:
        if matrix[2][1] < 0:
            ZYZ_angles[2] = 90
        else:
            if matrix[2][1] > 0:
                ZYZ_angles[2] = -90
            else:
                ZYZ_angles[2] = 0
    else:
        ZYZ_angles[2] = mt.degrees(mt.atan2(matrix[2][1], - matrix[2][0]))
    return ZYZ_angles

def matrix_to_ZYX(matrix):
    # converts a rotation matrix to ZYX tait-bryan angles witch correspond to yaw, pitch and roll (W, P, R)
    # it returns only one of the two solutions, the other one must be taken into consideration outside of this function
    ZYX_angles = [0.0, 0.0, 0.0]

    if mt.fabs(matrix[0][0]) < 0.00000001:
        if matrix[1][0] < -0.00000001:
            ZYX_angles[0] = -90
        else:
            if matrix[1][0] > 0.00000001:
                ZYX_angles[0] = 90
            else:
                ZYX_angles[0] = 0
    else:
        ZYX_angles[0] = mt.degrees(mt.atan2(matrix[1][0], matrix[0][0]))

    if matrix[2][0] > 0.999999999:
        ZYX_angles[1] = -90
    else:
        if matrix[2][0] < -0.999999999:
            ZYX_angles[1] = 90
        else:
            ZYX_angles[1] = mt.degrees(mt.atan2(-matrix[2][0], mt.sqrt(1 - mt.pow(matrix[2][0], 2))))

    if mt.fabs(matrix[2][2]) < 0.00000001:
        if matrix[2][1] < -0.00000001:
            ZYX_angles[2] = -90
        else:
            if matrix[2][1] > 0.00000001:
                ZYX_angles[2] = 90
            else:
                ZYX_angles[2] == 0
    else:
        ZYX_angles[2] = mt.degrees(mt.atan2(matrix[2][1], matrix[2][2]))

    return ZYX_angles

def ZYZ_to_matrix(angles):
    # converts ZYZ euler angles to a rotation matrix
    cos = [mt.cos(mt.radians(angles[0])), mt.cos(mt.radians(angles[1])), mt.cos(mt.radians(angles[2]))]
    sin = [mt.sin(mt.radians(angles[0])), mt.sin(mt.radians(angles[1])), mt.sin(mt.radians(angles[2]))]
    return np.array([[cos[0] * cos[1] * cos[2] - sin[0] * sin[2], -cos[2] * sin[0] - cos[0] * cos[1] * sin[2], cos[0] * sin[1]], [cos[0] * sin[2] + cos[1] * cos[2] * sin[0], cos[0] * cos[2] - cos[1] * sin[0] * sin[2], sin[0] * sin[1]], [-cos[2] * sin[1], sin[1] * sin[2], cos[1]]])

def ZYX_to_matrix(angles):
    # converts ZYX tait-bryan angles to a rotation matrix
    cos = [mt.cos(mt.radians(angles[0])), mt.cos(mt.radians(angles[1])), mt.cos(mt.radians(angles[2]))]
    sin = [mt.sin(mt.radians(angles[0])), mt.sin(mt.radians(angles[1])), mt.sin(mt.radians(angles[2]))]
    return np.array([[cos[0] * cos[1], sin[1] * sin[2] * cos[0] - cos[2] * sin[0], sin[2] * sin[0] + cos[0] * cos[2] * sin[1]], [cos[1] * sin[0], cos[0] * cos[2] + sin[0] * sin[1] * sin[2], cos[2] * sin[0] * sin[1] - cos[0] * sin[2]], [-sin[1], cos[1] * sin[2], cos[1] * cos[2]]])

def invert_ZYZ(angles):
    #returns the second solution for ZYZ euler angles
    angles2 = [0.0, -angles[1], 0.0]
    if angles[0] > 0:
        angles2[0] = angles[0] - 180
    else:
        angles2[0] = angles[0] + 180

    if angles[2] > 0:
        angles2[2] = angles[2] - 180
    else:
        angles2[2] = angles[2] + 180

    return angles2

def invert_ZYX(angles):
    angles2 = [0.0, 0.0, 0.0]
    if angles[0] > 0:
        angles2[0] = angles[0] - 180
    else:
        angles2[0] = angles[0] + 180

    if angles[1] > 0:
        angles2[1] = -angles[1] + 180
    else:
        angles2[1] = -angles[1] - 180

    if angles[2] > 0:
        angles2[2] = angles[2] - 180
    else:
        angles2[2] = angles[2] + 180

    return angles2

def get_transition_matrix(position):
    #transforms translation vector and ZYX euler angles into an homogenous transition matrix 
    rot_matrix = ZYX_to_matrix(position[1])
    return np.array([np.append(rot_matrix[0], position[0][0]), np.append(rot_matrix[1], position[0][1]), np.append(rot_matrix[2], position[0][2]), [0, 0, 0, 1]])

def position_distance(pos1, pos2):
    #used to estimate how far the robot is from the goal position at the end of a move
    m1 = get_transition_matrix(pos1)
    m2 = get_transition_matrix(pos2)
    s = 0
    coefs = [5.0, 5.0, 5.0, 2]
    for i in range(3):
        for j in range(4):
            s += coefs[j] * pow(m1[i][j] - m2[i][j], 2)
    return s

def smaller_rotation(rot1, rot2):
    #returns the euler angles configuration with the smallest square sum
    if mt.pow(rot1[0], 2) + mt.pow(rot1[1], 2) + mt.pow(rot1[2], 2) < mt.pow(rot2[0], 2) + mt.pow(rot2[1], 2) + mt.pow(rot2[2], 2):
        return rot1
    return rot2


def forward_kinematics(joint_angles, tool_offset):
    #computes the position of the tool, knowing the joints angle (AKA the easy part)
    position = {"faisability" : True, "joint_angles" : joint_angles, "tool_position" : [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], "wrist_position" : [0.0, 0.0, 0.0], "world_tool_matrix" : np.array([])}
    if is_not_faisable_A(joint_angles):
        position["faisability"] = False
        return position
    #pre-calculating the cos and sin since they will be used a lot
    angle_cos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    angle_sin = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(6):
        angle_cos[i] = mt.cos(mt.radians(joint_angles[i]))
        angle_sin[i] = mt.sin(mt.radians(joint_angles[i]))
    #defining homogenous transition matrix for every joint
    #the last one has the tool offset built-in
    tr_matrix_0 = np.array([[angle_cos[0], -angle_sin[0], 0.0, 0.0], [angle_sin[0], angle_cos[0], 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    tr_matrix_1 = np.array([[angle_cos[1], 0.0, angle_sin[1], angle_sin[1] * 150], [0.0, 1.0, 0.0, 0.0], [-angle_sin[1], 0.0, angle_cos[1], angle_cos[1] * 150], [0.0, 0.0, 0.0, 1.0]])
    tr_matrix_2 = np.array([[-angle_sin[2], 0.0, angle_cos[2], angle_cos[2] * 150], [0.0, 1.0, 0.0, 0.0], [-angle_cos[2], 0.0, -angle_sin[2], -angle_sin[2] * 150], [0.0, 0.0, 0.0, 1.0]])
    tr_matrix_3 = np.array([[angle_cos[3], -angle_sin[3], 0.0, 0.0], [angle_sin[3], angle_cos[3], 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    tr_matrix_4 = np.array([[angle_cos[4], 0.0, angle_sin[4], 0.0], [0.0, 1.0, 0.0, 0.0], [-angle_sin[4], 0.0, angle_cos[4], 0.0], [0.0, 0.0, 0.0, 1.0]])
    tr_matrix_5 = np.array([[angle_cos[5], -angle_sin[5], 0.0, tool_offset[0] * angle_cos[5] - tool_offset[1] * angle_sin[5]], [angle_sin[5], angle_cos[5], 0.0, angle_sin[5] * tool_offset[0] + angle_cos[5] * tool_offset[1]], [0.0, 0.0, 1.0, tool_offset[2]], [0.0, 0.0, 0.0, 1.0]])
    #multiplying the matrix makes magic
    position["world_tool_matrix"] = np.dot(tr_matrix_0, tr_matrix_1)
    position["world_tool_matrix"] = np.dot(position["world_tool_matrix"], tr_matrix_2)
    #at this point we have wrist center coordinates
    position["wrist_position"] = [position["world_tool_matrix"][0][3], position["world_tool_matrix"][1][3], position["world_tool_matrix"][2][3]]
    #resume matrix multiplication
    position["world_tool_matrix"] = np.dot(position["world_tool_matrix"], tr_matrix_3)
    position["world_tool_matrix"] = np.dot(position["world_tool_matrix"], tr_matrix_4)
    position["world_tool_matrix"] = np.dot(position["world_tool_matrix"], tr_matrix_5)
    # world_tool_matrix is done, so we got everything we need
    tool_rotation = matrix_to_ZYX(position["world_tool_matrix"])
    tool_rotation_2 = invert_ZYX(tool_rotation)
    position["tool_position"] = [[position["world_tool_matrix"][0][3], position["world_tool_matrix"][1][3], position["world_tool_matrix"][2][3]], smaller_rotation(tool_rotation, tool_rotation_2)]
    if is_not_faisable_P(position):
        position["faisability"] = False
    return position

def is_not_faisable_A(angles):
    mins = [-90.0, -52.5, -144.6, -180.0, -135.0, -180.0]
    maxs = [90.0, 90.0, 50.0, 180.0, 135.0, 180.0]
    #joint beyond their limits
    for i in range(6):
        if angles[i] < mins[i] or angles[i] > maxs[i]:
            return True
    #interaction J3 / J4
    if angles[3] > 110 and angles[2] < -51:
        return True
    if angles[3] < -110 and angles[2] < -55:
        return True
    if angles[2] > 15 and angles[3] < -120:
        return True
    if angles[2] > 23 and angles[3] > 120:
        return True
    return False

def is_not_faisable_P(position):
    #crash in the floor
    if position["wrist_position"][2] < -150:
        return True
    if position["tool_position"][0][2] < -180:
        return True
    #crash into the base
    Z_axis_distance_W = mt.sqrt(mt.pow(position["wrist_position"][0], 2) + mt.pow(position["wrist_position"][1], 2))
    Z_axis_distance_T = mt.sqrt(mt.pow(position["tool_position"][0][0], 2) + mt.pow(position["tool_position"][0][1], 2))
    if Z_axis_distance_W < 80 and position["wrist_position"][2] < 20:
        return True
    if Z_axis_distance_T < 80 and position["tool_position"][0][2] < 20:
        return True
    return False

def servo_movement(joint_angles_1, joint_angles_2):
    # used to find the arm configuration that minimise the movement of the joints.
    Coefs = [60, 100, 50, 10, 10, 10]
    score = 0
    for i in range(6):
        score += Coefs[i] * mt.pow(joint_angles_1[i] - joint_angles_2[i], 2)
    # we want this function to be biased in favor of angles with a small angle for joint 4 to reduce  the need for colision avoidance 
    score += mt.pow(joint_angles_1[3], 2) * 25
    return score

def inverse_kinematics(tool_position, tool_offset, last_joint_angles):
    #computes the joints angles given a desired position of the tool (AKA the real shit)
    #since there are 8 possibilities to reach a given position, it makes a list of all possibilities
    #exept when there is a singularity, since there would be an infinite number of solutions
    #then it just sets the joint causing the singularity to its last value
    #at the end we remove the possibilities that can not be reached
    #and we select the one witch minimize servo movements (least square)
    angle_options = list()
    singularity = [False, False, False, False, False]
    for i in range(8):
        angle_options.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    position = {"faisability" : True, "joint_angles" : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "tool_position" : tool_position, "wrist_position" : [0.0, 0.0, 0.0], "world_tool_matrix" : get_transition_matrix(tool_position)}
    #first we calculate wrist center position from tool position, orientation and offset
    position["wrist_position"][0] = tool_position[0][0] - tool_offset[0] * position["world_tool_matrix"][0][0] - tool_offset[1] * position["world_tool_matrix"][0][1] - tool_offset[2] * position["world_tool_matrix"][0][2]
    position["wrist_position"][1] = tool_position[0][1] - tool_offset[0] * position["world_tool_matrix"][1][0] - tool_offset[1] * position["world_tool_matrix"][1][1] - tool_offset[2] * position["world_tool_matrix"][1][2]
    position["wrist_position"][2] = tool_position[0][2] - tool_offset[0] * position["world_tool_matrix"][2][0] - tool_offset[1] * position["world_tool_matrix"][2][1] - tool_offset[2] * position["world_tool_matrix"][2][2]
    square_sum = mt.pow(position["wrist_position"][0], 2) + mt.pow(position["wrist_position"][1], 2) + mt.pow(position["wrist_position"][2], 2)
    if square_sum > 90000 or is_not_faisable_P(position):
        position["faisability"] = False
        return position
    #the first 3 joints angles can be found using a paper, pen and basic trigonometry, there are specific cases to be taken into account though
    value = 0
    if mt.fabs(position["wrist_position"][0]) < 0.000001:
        if mt.fabs(position["wrist_position"][1]) < 0.000001:
            #singularity, there is a solution for every position of the first joint, so we just set its value to the last one it had
            for i in range(8):
                angle_options[i][0] = last_joint_angles[0]
            #setting the value of the first angle reduces to 4 the number of possibilities but we will take care of that at the end
            singularity[0] = True
        else:
            if position["wrist_position"][1] < 0:
                value = -90
            else:
                value = 90
    else:
        value = mt.degrees(mt.atan2(position["wrist_position"][1], position["wrist_position"][0]))
    if not singularity[0]:
        #normal shoulder
        for i in range(4):
            angle_options[i][0] = value
        #inverted shoulder
        if value < 0:
            value += 180
        else:
            value -= 180
        for i in range(4, 8):
            angle_options[i][0] = value
    #definition of angles from the geometric construction that allows solving joint 2 and 3
    dist_xy = mt.sqrt(pow(position["wrist_position"][0], 2) + pow(position["wrist_position"][1], 2))
    phi = 0
    if dist_xy < 0.000001:
        if position["wrist_position"][2] < 0:
            phi = -90
        else:
            phi = 90
    else:
        phi = mt.degrees(mt.atan2(position["wrist_position"][2], dist_xy))
    psi = mt.degrees(mt.acos(mt.sqrt(square_sum) / 300))
    #solving of joints 2 and 3, four option are possible
    #normal shoulder, normal elbow
    angle_options[0][1] = 90.0 - phi - psi
    angle_options[0][2] = 2.0 * psi - 90
    angle_options[1][1] = 90.0 - phi - psi
    angle_options[1][2] = 2.0 * psi - 90
    #normal shoulder, inverted elbow
    angle_options[2][1] = 90.0 - phi + psi
    angle_options[2][2] = -180.0 - angle_options[0][2]
    angle_options[3][1] = 90.0 - phi + psi
    angle_options[3][2] = -180.0 - angle_options[0][2]
    #inverted shoulder, normal elbow
    angle_options[4][1] = phi + psi - 90
    angle_options[4][2] = angle_options[2][2]
    angle_options[5][1] = phi + psi - 90
    angle_options[5][2] = angle_options[2][2]
    #inverted shoulder, inverted elbow
    angle_options[6][1] = phi - psi - 90
    angle_options[6][2] = angle_options[0][2]
    angle_options[7][1] = phi - psi - 90
    angle_options[7][2] = angle_options[0][2]
    #now that we have the first 3 joints angles, we can get the orientation of the wrist in each case and then compute the 3 last joints angles (for each of the four cases)
    rot_matrix_W_T = ZYX_to_matrix(tool_position[1])
    for i in range(4):
        angle_cos = np.array([mt.cos(mt.radians(angle_options[2 * i][0])), mt.cos(mt.radians(angle_options[2 * i][1])), mt.cos(mt.radians(angle_options[2 * i][2]))])
        angle_sin = np.array([mt.sin(mt.radians(angle_options[2 * i][0])), mt.sin(mt.radians(angle_options[2 * i][1])), mt.sin(mt.radians(angle_options[2 * i][2]))])
        rot_matrix_0 = np.array([[angle_cos[0], -angle_sin[0], 0.0], [angle_sin[0], angle_cos[0], 0.0], [0.0, 0.0, 1.0]])
        rot_matrix_1 = np.array([[angle_cos[1], 0.0, angle_sin[1]], [0.0, 1.0, 0.0], [-angle_sin[1], 0.0, angle_cos[1]]])
        rot_matrix_2 = np.array([[-angle_sin[2], 0.0, angle_cos[2]], [0.0, 1.0, 0.0], [-angle_cos[2], 0.0, -angle_sin[2]]])
        rot_matrix_W_3 = np.dot(rot_matrix_0, rot_matrix_1)
        rot_matrix_W_3 = np.dot(rot_matrix_W_3, rot_matrix_2)

        # now that we got the the rotation matrix from world to tool and from world to wrist, we can get the one from wrist to tool
        rot_matrix_3_T = np.linalg.inv(rot_matrix_W_3)
        rot_matrix_3_T = np.dot(rot_matrix_3_T, rot_matrix_W_T)
        #and then the joints angles correspond to the euler angles
        #normal wrist

        wrist_euler_angles = matrix_to_ZYZ(rot_matrix_3_T)
        if mt.fabs(wrist_euler_angles[1]) < .0001 or mt.fabs(wrist_euler_angles[1] - 180) < 0.000001:
            #wrist singularity
            singularity[i + 1] = True
            angle_options[2 * i][3] = last_joint_angles[3]
            angle_options[2 * i][4] = wrist_euler_angles[1]
            angle_options[2 * i][5] = wrist_euler_angles[0] - last_joint_angles[3]
        else:
            angle_options[2 * i][3] = wrist_euler_angles[0]
            angle_options[2 * i][4] = wrist_euler_angles[1]
            angle_options[2 * i][5] = wrist_euler_angles[2]
            #inverted wrist
            wrist_euler_angles = invert_ZYZ(wrist_euler_angles)
            angle_options[2 * i + 1][3] = wrist_euler_angles[0]
            angle_options[2 * i + 1][4] = wrist_euler_angles[1]
            angle_options[2 * i + 1][5] = wrist_euler_angles[2]

    #we now have all the 8 possibilities, first we remove the ones that might be doubled due to singularities
    if singularity[4]:
        angle_options.remove(angle_options[7])
    if singularity[3]:
        angle_options.remove(angle_options[5])
    if singularity[2]:
        angle_options.remove(angle_options[3])
    if singularity[1]:
        angle_options.remove(angle_options[1])
    if singularity[0]:
        while len(angle_options) > 4:
            angle_options.remove(angle_options[4])
    # now we remove the ones that are not possible for some reason

    c = 0
    while c < len(angle_options):
        if is_not_faisable_A(angle_options[c]):
            angle_options.remove(angle_options[c])
            c -= 1
        c += 1
    # if the list is now empty, then we can not reach the desired position
    if len(angle_options) == 0:
        position["faisability"] = False
        return position
    #finnaly, we select the angles that minimize servo movement from the last position
    position["joint_angles"] = angle_options[0]
    if len(angle_options) == 0:
        return position
    best_score = servo_movement(angle_options[0], last_joint_angles)
    for i in range(1, len(angle_options)):
        score = servo_movement(angle_options[i], last_joint_angles)
        if score < best_score:
            best_score = score
            position["joint_angles"] = angle_options[i]
    return position

def rotate(pos, matrix):
    # returns a position rotated by a rotation matrix
    pos_matrix = ZYX_to_matrix(pos[1])
    pos_matrix = np.dot(matrix, pos_matrix)

    pos1 = [pos[0], matrix_to_ZYX(pos_matrix)]
    pos2 = [pos[0], invert_ZYX(pos1[1])]

    if position_distance(pos1, pos) < position_distance(pos2, pos):
        return pos1
    return pos2

def get_rot_matrix(theta, axis):
    # returns a rotation matrix for the rotation around X, Y or Z by an angle theta
    if axis == "X" or axis == "x":
        return np.array([[1.0, 0.0, 0.0], [0.0, mt.cos(mt.radians(theta)), -mt.sin(mt.radians(theta))], [0.0, mt.sin(mt.radians(theta)), mt.cos(mt.radians(theta))]])
    if axis == "Y" or axis == "x":
        return np.array([[mt.cos(mt.radians(theta)), 0.0, mt.sin(mt.radians(theta))], [0.0, 1.0, 0.0], [-mt.sin(mt.radians(theta)), 0.0, mt.cos(mt.radians(theta))]])
    if axis == "Z" or axis == "x":
        return np.array([[mt.cos(mt.radians(theta)), -mt.sin(mt.radians(theta)), 0.0], [mt.sin(mt.radians(theta)), mt.cos(mt.radians(theta)), 0.0], [0.0, 0.0, 1.0]])
    return np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

def get_rot_matrix_2(m1, m2):
    # get the rotation matrix that make the transition between the rotation matrix m1 and m2
    inv_m1 = np.linalg.inv(m1)
    return np.dot(inv_m1, m2)

def axis_angle_to_matrix(x, y, z, theta):
    # convert a unit vector and an angle to a matrix corresponding to rotating around the vector by the angle
    c = mt.cos(mt.radians(theta))
    s = mt.sin(mt.radians(theta))
    C = 1 - c
    return np.array([[x*x*C + c, x*y*C - z*s, x*z*C + y*s], [y*x*C + z*s, y*y*C + c, y*z*C - x*s], [z*x*C - y*s, z*y*C + x*s, z*z*C + c]])

def matrix_to_axis_angle(matrix):
    #algorithm taken from euclidianspace.com
    #computes an axis vector and a rotation angle from a rotation matrix
    angle = 0.0
    x = 0.0
    y = 0.0
    z = 0.0
    if mt.fabs(matrix[0][1] - matrix[1][0]) < 0.00001 and mt.fabs(matrix[0][2] - matrix[2][0]) < 0.00001 and mt.fabs(matrix[1][2] - matrix[2][1]) < 0.00001:
        #singularity
        if mt.fabs(matrix[0][1] - matrix[1][0]) < 0.00001 and mt.fabs(matrix[0][2] - matrix[2][0]) < 0.00001 and mt.fabs(matrix[1][2] - matrix[2][1]) < 0.00001 and mt.fabs(matrix[0][0] + matrix[1][1] + matrix[2][2] - 3) < 0.00001:
            # singularity of 0 degrees angle, vector is irrelevant
            return [0.0, 1.0, 0.0, 0.0]
        else:
            angle = 180.0
            xx = (matrix[0][0] + 1) / 2
            yy = (matrix[1][1] + 1) / 2
            zz = (matrix[2][2] + 1) / 2
            xy = (matrix[0][1] + matrix[1][0]) / 4
            xz = (matrix[0][2] + matrix[2][1]) / 4
            yz = (matrix[1][2] + matrix[2][1]) / 4
            if xx > yy and xx > zz:
                if xx < 0.00001:
                    x = 0.0
                    y = mt.sqrt(2) / 2
                    z = mt.sqrt(2) / 2
                else:
                    x = mt.sqrt(xx)
                    y = xy / x
                    z = xz / x
                return [angle, x, y, z]
            if yy > zz:
                if yy < 0.00001:
                    x = mt.sqrt(2) / 2
                    y = 0.0
                    z = mt.sqrt(2) / 2
                else:
                    y = mt.sqrt(yy)
                    x = xy / y
                    z = yz / y
                return [angle, x, y, z]
            if zz < 0.00001:
                x = mt.sqrt(2) / 2
                y = mt.sqrt(2) / 2
                z = 0.0
            else:
                z = mt.sqrt(zz)
                x = xz / z
                y = yz / z
            return [angle, x, y, z]
    else:
        s = mt.sqrt(mt.pow(matrix[2][1] - matrix[1][2], 2) + mt.pow(matrix[0][2] - matrix[2][0], 2) + mt.pow(matrix[1][0] - matrix[0][1], 2))
        angle = mt.degrees(mt.acos((matrix[0][0] + matrix[1][1] + matrix[2][2] - 1) / 2))
        x = (matrix[2][1] - matrix[1][2]) / s
        y = (matrix[0][2] - matrix[2][0]) / s
        z = (matrix[1][0] - matrix[0][1]) / s
        return [angle, x, y, z]
