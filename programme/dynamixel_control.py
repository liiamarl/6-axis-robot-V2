from dynamixel_sdk import *
from time import sleep


#mostly copy and paste from the examples of the dynamixelSDK library


port = PortHandler("/dev/ttyUSB0")
packetHandler = PacketHandler(2.0)

def setup_ports():
    global port, packetHandler1
    if port.openPort():
        print("Succeeded to open port")
    else:
        print("Failed to open port")
        quit()

    if port.setBaudRate(4000000):
        print("Succeeded to change the baudrate for port1")
    else:
        print("Failed to change the baudrate for port1")
        quit()

def enable_torque():
    global port, packetHandler

    for i in range(6):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(port, i + 11, 64, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel ", i + 1, " has been successfully connected")


def disable_torque():
    global port1, packetHandler


    for i in range(6):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(port, i + 11, 64, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel ", i + 1, " has been successfully deconnected")



def move_servo(ID, position):
    global port, packetHandler
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(port, ID, 116, position)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def get_servo_position(ID):
    global port, packetHandler
    position = -1
    position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(port, ID, 132)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return position

def write_data(ID, adress, lenth, data):
    global port, packetHandler
    if lenth == 1:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(port, ID, adress, data)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    if lenth == 2:
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(port, ID, adress, data)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    if lenth == 4:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(port, ID, adress, data)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))





def read_data(ID, adress, lenth):
    global port, packetHandler
    if lenth == 1:
        data, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(port, ID, adress)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler1.getRxPacketError(dxl_error))
    if lenth == 2:
        data, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(port, ID, adress)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler1.getRxPacketError(dxl_error))
    if lenth == 4:
        data, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(port, ID, adress)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler1.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler1.getRxPacketError(dxl_error))
    return data


    
