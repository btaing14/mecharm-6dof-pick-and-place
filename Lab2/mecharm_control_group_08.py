from time import sleep
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
import math
import csv
import sympy as sp


# Display the current end effector coordinates and the corresponding joint angles
def displayRobotStatus(mycobot: MyCobot):
    angles = mycobot.get_angles()
    coord = mycobot.get_coords()
    # Odd Line will be angles while Even will be coords
    print(angles)
    print(coord)

# Read the data.csv and return angles and coord; both in 2d array list from position 1 to 10
def readCSV():
    # Load Data
    filename = "data.csv"
    angles = []
    coord = []

    with open(filename, mode="r") as f:
        reader = csv.reader(f)         
        i = 0
        #The csv read comes in this format: ['[11.42', ' 119.35', ' 166.72', ' 133.94', ' -81.56', ' -62.84]']
        for row in reader:
            #Remove [ and ] from first index string and last one
            print()
            row[0] = row[0].replace('[', '')
            row[-1] = row[-1].replace(']', '')
            #Conver String to float
            temp = []
            for col in row:
                temp.append(float(col))

            #even Line of the csv is angle and odd is coord here, because i is 0-based
            if (i % 2 == 0):
                angles.append(temp)
            else:
                coord.append(temp)
            i+=1
    return angles, coord    


            

# Call this function to read 10 positions:
def Read10Position():
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()
    sleep(2)
    mycobot.release_all_servos()
    # Print the status of the robotic arm for 11 times (we're never ready to read the first output)
    for i in range(12):
        print("Position: ", i)
        displayRobotStatus(mycobot)
        print("\n")
        #5 second pause
        sleep(5)
    mycobot.power_off()

#Call this function to set position according to the angles
def setPositionFAngles():
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()
    #Get angles
    angles,_ = readCSV()
    #back to 0 position
    sleep(3)
    mycobot.send_angles([0,0,0,0,0,0],30) #Start at home position
    print("Start joint angles")
    sleep(3)
    mycobot.set_gripper_state(1,30)
    sleep(3)

    for i in range(5): #Go to each set of joint angles, 5 times
        print("Starting sequence iteration ", i+1)
        for j in range(len(angles)):
            print(angles[j])
            mycobot.send_angles(angles[j],30)
            sleep(5)
            mycobot.send_angles([0,0,0,0,0,0],50)
            sleep(3)

    mycobot.send_angles([0,0,0,0,0,0],30)
    sleep(3)
    mycobot.power_off()

#Call this function to set position according to the coord
def setPositionFCoord():
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()
    #Get angles
    _,coord = readCSV()
    #back to 0 position

    
    mycobot.send_angles([0,0,0,0,0,0],30)
    print("Start coordinates")
    sleep(3)
    mycobot.set_gripper_state(1,30)
    sleep(3)

    for i in range(5): #Go to each set of coordinates, 5 times
        print("Starting sequence iteration ", i+1)
        for j in range(len(coord)):
            print(coord[j])
            mycobot.send_coords(coord[j],30)
            sleep(5)
            mycobot.send_angles([0,0,0,0,0,0],50)
            sleep(3)

    mycobot.send_angles([0,0,0,0,0,0],30)
    sleep(3)
    mycobot.power_off()

#setPositionFAngles() #Uncomment this to test joint angles
#setPositionFCoord() #Uncomment this to test coordinates
#Read10Position() #Uncomment this to drag teach
