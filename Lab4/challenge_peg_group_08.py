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

mycobot = MyCobot(PI_PORT, PI_BAUD) #Create MyCobot instantiation
angles,coords = readCSV("peg.csv")
readyPick = [coords[0],angles[0]] #Constant position to hover the arm above the set pickup location
pickPosition = [coords[1], angles[1]] #Constant position to pick up a peg

#Pick up a peg from the set pickup location and return to neutral position
def pickUpPeg():
    mycobot.send_coords(getCoordinates(readyPick[0], mycobot.get_angles()), 30) #Inverse kinematics converges with less iterations if we use current angles as initial guess
    print("At ready to pick")
    sleep(2)
    mycobot.send_angles(pickPosition[1], 10)
    print("In position to pick")
    sleep(3)
    mycobot.set_gripper_value(40,30,1)
    print("Closed gripper")
    sleep(2)
    mycobot.send_coords(getCoordinates(readyPick[0], mycobot.get_angles()), 30)
    print("Back at ready to pick")
    sleep(2)
    mycobot.send_angles([0,0,0,0,0,0],30)
    print("Back to neutral")
    sleep(2)

#Place a peg into its slot by hovering the peg above the slot (stageCoord) then moving down into position (placeCoord). Returns back to neutral position
def placePeg(stageCoord, placeCoord):
    mycobot.send_coords(getCoordinates(stageCoord, mycobot.get_angles()), 30)
    print("in stage position")
    sleep(3)
    mycobot.send_coords(getCoordinates(placeCoord, mycobot.get_angles()), 10) #Place pegs very slowly
    print("In PLACE position")
    sleep(7)
    mycobot.set_gripper_state(0,30)
    print("opened gripper")
    sleep(4)
    mycobot.send_coords(getCoordinates(stageCoord, mycobot.get_angles()), 30)
    print("Back to stage position")
    sleep(3)
    mycobot.set_gripper_state(0,30)
    print("opened gripper again")
    sleep(4)
    mycobot.send_angles([0,0,0,0,0,0],30)
    print("Back to neutral")
    sleep(3)

#Performs inverse kinematics to get solution joint angles, and recalculates solution coordinates using our own forward kinematics
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

#Function to pick and place three pegs
def the_Challenge():
    # Initial our robot arm
    mycobot.power_on()

    # Set robot to default position with the gripper open
    sleep(3)
    mycobot.send_angles([0,0,0,0,0,0],30)
    print("Start coordinates")
    sleep(2)
    mycobot.set_gripper_state(0,30)
    print("gripper open")
    sleep(3)

    #Pick up and place 3 pegs
    pickUpPeg()
    placePeg(coords[2], coords[3])
    mycobot.set_gripper_state(0,30)
    print("gripper open")
    sleep(1)
    pickUpPeg()
    placePeg(coords[4], coords[5])
    mycobot.set_gripper_state(0,30)
    print("gripper open")
    sleep(1)
    pickUpPeg()
    placePeg(coords[6], coords[7])
    mycobot.send_angles([0,0,0,0,0,0],30) #End at neutral

def testPlace():
    mycobot.power_on()

    # Set robot to default position with the gripper open
    sleep(3)
    mycobot.send_angles([0,0,0,0,0,0],30)
    print("Start coordinates")
    sleep(2)
    mycobot.set_gripper_state(0,30) #Open gripper to grab a peg
    print("gripper open")
    sleep(3)
    mycobot.set_gripper_value(40,30,1) #Grab the peg
    print("Closed gripper")
    sleep(2)
    placePeg(coords[4], coords[5]) #Set of peg place coordinates to test

#Read10Position() #Uncomment this to drag teach

#testPlace() #Uncomment this to test placing a single peg into its slot starting from neutral
the_Challenge() #Uncomment this to run Lab 4 Challenge 1
