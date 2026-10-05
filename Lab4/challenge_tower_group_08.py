import sys
import os

# With sys Get the path of the project root which is the EE 347 Labs folder where it hold folder of Lab2 and Lab3 and Lab4... Citation: https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir 
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)

# Import function from Lab2 and 3
from Lab3.mecharm_INVK_advanced_group_08 import *   # or specific functions
from Lab2.mecharm_control_group_08 import Read10Position

import csv
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD


def sendRobotToSpecificLocation(angles, coords, readyPick, readyDrop):
    # Initial our robot arm
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()

    # Set robot to default position with the gripper open
    sleep(2)
    mycobot.send_angles([0,0,0,0,0,0],40)
    print("Start coordinates")
    sleep(2)
    mycobot.set_gripper_state(0,40)
    sleep(1)

    states = 1
    blockPick = 0
    blockDrop = 0
    for i in range(len(coords)):
        if (states == 1):
            print(getCoordinates(readyPick[blockPick][0], mycobot.get_angles()))
            mycobot.send_coords(getCoordinates(readyPick[blockPick][0], mycobot.get_angles()), 30)
            print("Go to ready to pick states")
            sleep(2)

        elif (states == 2):
            mycobot.send_coords(getCoordinates(readyDrop[blockDrop][0], mycobot.get_angles()), 30)
            print("Go to ready to drop states")
            sleep(2)

        end_effector_position = getCoordinates(coords[i], mycobot.get_angles())
        print(end_effector_position)
        mycobot.send_coords(end_effector_position, 20)
        print("Go to pick/drop states")
        sleep(3)

        if (states == 1):
            mycobot.set_gripper_value(70,30,1)
            print("pick up")
            sleep(2)
            mycobot.send_coords(getCoordinates(readyPick[blockPick][0], mycobot.get_angles()), 50)
            blockPick+=1
            print("Go to ready to pick states")
            sleep(2)
            states = 2
        elif (states == 2):
            mycobot.set_gripper_state(0,20)
            print("Drop")
            sleep(2)
            mycobot.send_coords(getCoordinates(readyDrop[blockDrop][0], mycobot.get_angles()), 30)
            blockDrop+=1
            print("Go to ready to drop states")
            sleep(2)
            states = 1


        print("Go to Neutral States")
        mycobot.send_angles([0,0,0,0,0,0],30)
        sleep(2)



def getCoordinates(coords, angles):
    x_target = coords[0]
    y_target = coords[1]
    z_target = coords[2]
    rx_d = np.radians(coords[3])  # Roll angle (in radians)
    ry_d = np.radians(coords[4])  # Pitch angle (in radians)
    rz_d = np.radians(coords[5])  # Yaw angle (in radians)
    q_init = angles
    joint_angles = ik((x_target, y_target, z_target, rx_d, ry_d, rz_d), q_init) # apply inverse kinematic to find computed angles
    calculated_coords = symbolic_forward_kinematics(joint_angles) # apply forward kinematics to find the computed coordinates with computed angles
    # Since the coordinates that we got only contains x y z... we then add the "recorded" rx ry rz
    end_effector_position = calculated_coords[:3, 3][0:3] + [coords[3], coords[4], coords[5]]
    # Send coordinates to robot
    return end_effector_position

def blockPickNDrop():
    # CSV's first two line will be ready Pick zone... second two line will be ready drop coordinates
    angles,coords = readCSV("block.csv")
    readyPick = [[coords[0],angles[0]],[coords[1],angles[1]],[coords[2],angles[2]]]
    readyDrop = [[coords[3],angles[3]],[coords[4],angles[4]],[coords[5],angles[5]]]
    sendRobotToSpecificLocation(angles[6:], coords[6:], readyPick,readyDrop)

def test():
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()

    sleep(2)

    mycobot.send_angles([0,0,0,0,0,0],30)
    sleep(2)
    mycobot.send_coords([165.8, -67.0, 150.4, 111.45, 76.79, 131.24],20)
    sleep(2)
    mycobot.send_coords([165.8, -67.0, 110.4, 111.45, 76.79, 131.24],20)

    
    sleep(2)
    
#[188.0, -71.0, 152.5, 152.36, 84.35, 130.6]
#Read10Position() #Uncomment this to drag teach

blockPickNDrop() #Run Challenge 2: deconstruct and rebuild the block tower
#test() #Uncomment to test a single hard-coded move
