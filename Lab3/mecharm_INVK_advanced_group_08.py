from time import sleep
import numpy as np
from sympy  import symbols, cos, sin, atan2, pi, Matrix, lambdify 
from scipy.optimize import least_squares
import csv


from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD

# Declare symbols
q1,q2,q3,q4,q5,q6 = symbols('q1 q2 q3 q4 q5 q6')

# Define the offset parameters
offsets = [0, 0, 0, 0, 0, 0]

# Declare the DH Table
D_H_Table = [[0,0,114,q1],
            [0,-90,0,q2-pi/2],
            [100,0,0,q3],
            [10,-90,95,q4],
            [0,90,0,q5],
            [0,-90,58,q6]]

# Helper function to take the D-H table row values as input and outputs the corresponding transformation matrix
def get_transformation_matrix(a, alpha, d, theta):
    # Convert our alpha from degree to radian
    alpha = np.radians(alpha)
    M = Matrix([[cos(theta), -1 * sin(theta), 0, a],
                [sin(theta)*np.cos(alpha),cos(theta)*np.cos(alpha),-1 * np.sin(alpha),-1 * np.sin(alpha)*d],
                [sin(theta)*np.sin(alpha),cos(theta)*np.sin(alpha),np.cos(alpha),np.cos(alpha)*d],
                [0,0,0,1]])
    return M

# Calculate overall transformation matrix
T06 = get_transformation_matrix(D_H_Table[0][0],D_H_Table[0][1],D_H_Table[0][2],D_H_Table[0][3])
for i in range(1, len(D_H_Table)):
    T06 = T06 @ get_transformation_matrix(D_H_Table[i][0], D_H_Table[i][1], D_H_Table[i][2], D_H_Table[i][3])

# Define symbolic joint variables for differentiation
q_sym = symbols('q1:7')

# With forward kinemetics, we're getting the coordinates with given angles values.... return the coordinates of end effector
def symbolic_forward_kinematics(q_values):
    # Map the joint angles and offsets into the transformation matrix
    subs_dict = {q:offset + angle for q, offset, angle in zip([q1, q2, q3, q4, q5, q6], offsets, q_values)}
    T_symbolic = T06.subs(subs_dict)
    return T_symbolic

forward_kinematics_func = lambdify(q_sym, symbolic_forward_kinematics(q_sym), 'numpy')

# Converts the transformation matrix into a pose vector: [X, Y, Z, roll, pitch, yaw]
def transf_to_pose(t_matrix):
    # Position (fill in indices based on your matrix structure)
    X, Y, Z = t_matrix[0][3], t_matrix[1][3], t_matrix[2][3]


   # Breakdown the rotation matrix into axes
    roll = np.arctan2(t_matrix[2][1], t_matrix[2][2])
    pitch = np.arctan2(-t_matrix[2][0], np.sqrt((t_matrix[0][0] ** 2) + (t_matrix[1][0] ** 2)))
    yaw = np.arctan2(t_matrix[1][0],t_matrix[0][1])

    return X, Y, Z, roll, pitch, yaw

# Checks the current joint angles against the target pose and returns their delta
def target_pose_error(joint_angles, *args):
   target_pose = args[0]
   current_fk = forward_kinematics_func(*joint_angles)

   # Convert translation matrix to pose vector
   current_pose = transf_to_pose(current_fk)

   error = np.array(current_pose) - np.array(target_pose)
   return error

# Looks for a solution to inverse kinematics for a target pose and returns a transformation matrix
def ik(target_pose, init_pose, max_iter=1000, tolerance=1e-5, bounds=(-180, 180)):

   result = least_squares(target_pose_error, init_pose, args=(target_pose,), method='trf', max_nfev=max_iter, ftol=tolerance, bounds=bounds)

   if result.success:
       print(f"Inverse kinematics converged after {result.nfev} function evaluations.")
       return result.x
   else:
       print("Inverse kinematics did not converge.")
       return None

# Reading the file and return the recorded angles and coordinates. Again, even line of the pickNplace.csv/data.csv is angles and odd is the coordinates
# The pickNplace.csv contains coordinates and angles where the starting point of marker and ending point of marker.... there is a helper states/coordinates where
# the end-effector will first move to so that it wouldn't knock down the marker at starting point.
def readCSV(filename):
    # Load Data
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

# Reading the recorded angles + coordinates.... feed the those data in to the inverse kinematic to get the computed joint angles and feed them to forward kinematics
# Purpose: to ensure the computed angles using inverse kinematic will reflect on the exact coordinates recorded.
def test_with_recorded_coords():
    # Get angles and coordinates from file
    print("READING CSV")
    angles, coords = readCSV("../Lab2/data.csv")

    # Perform inverse kinematics
    print("PERFORMING INVERSE KINEMATICS")
    for i in range(len(angles)):
        # Cut out the x y z and roll pitch yaw from each coordinates list; get the angles too.
        x_target = coords[i][0]
        y_target = coords[i][1]
        z_target = coords[i][2]
        rx_d = np.radians(coords[i][3])  # Roll angle (in radians)
        ry_d = np.radians(coords[i][4])  # Pitch angle (in radians)
        rz_d = np.radians(coords[i][5])  # Yaw angle (in radians)
        q_init = angles[i]
        # With inverse kinematic we find the joint angles
        joint_angles = ik((x_target, y_target, z_target, rx_d, ry_d, rz_d), q_init)
        # Apply forward kinematics with computed angles to check if computed coordinates is an exact match on recorded coordinates
        # Un comment the couple line below to see if computed coordinates is same exact as recorded one (we keep it comment because the deliverables want us to print angles)

        calculated_coords = symbolic_forward_kinematics(joint_angles)
        end_effector_position = calculated_coords[:3, 3]
        # Print all coordinates out
        # Un comment the couple line below to see if computed coordinates is same exact as recorded one (we keep it comment because the deliverables want us to print angles)
        #print("Coor:")
        #for j in range(len(end_effector_position)):
        #    print("c" + str(j) + ": " + str(end_effector_position[j]))
                    
        # Un comment the couple line below to see the computed angles by inverse kinematic
        print("Angles:")
        for j in range(len(joint_angles)):
            print("c" + str(j) + ": " + str(joint_angles[j]))

# Function that control the robot to pick up a marker in a specific start position and then place it in the specific end position.
def pickNPlace():
    # Initial our robot arm
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()

    # Get angles
    angles,coords = readCSV("pickNplace.csv")

    # Set robot to default position with the gripper open
    sleep(3)
    mycobot.send_angles([0,0,0,0,0,0],30)
    print("Start coordinates")
    sleep(3)
    mycobot.set_gripper_state(0,30)
    sleep(3)

   # Find compute angles with inverse kinematics and then use forward kinematics to find the coordinates position at readytopick position (so we don't knock down the marker)
    x_target = coords[0][0]
    y_target = coords[0][1]
    z_target = coords[0][2]
    rx_d = np.radians(coords[0][3])  # Roll angle (in radians)
    ry_d = np.radians(coords[0][4])  # Pitch angle (in radians)
    rz_d = np.radians(coords[0][5])  # Yaw angle (in radians)
    q_init = angles[0]
    joint_angles = ik((x_target, y_target, z_target, rx_d, ry_d, rz_d), q_init) # apply inverse kinematic to find computed angles
    calculated_coords = symbolic_forward_kinematics(joint_angles) # apply forward kinematics to find the computed coordinates with computed angles
    # Since the coordinates that we got only contains x y z... we then add the "recorded" rx ry rz
    end_effector_position = calculated_coords[:3, 3][0:3] + [coords[0][3], coords[0][4], coords[0][5]]
    # Send coordinates to robot
    mycobot.send_coords(end_effector_position, 10)
    print("ReadytoPick States")
    sleep(5)
    # View the current coordinate we at
    print("Coor:")
    for j in range(len(end_effector_position)):
        print("c" + str(j) + ": " + str(end_effector_position[j]))


    # Find compute angles with inverse kinematics and then use forward kinematics to find the coordinates position at starting position(where the marker is at)
    x_target = coords[1][0]
    y_target = coords[1][1]
    z_target = coords[1][2]
    rx_d = np.radians(coords[1][3])  # Roll angle (in radians)
    ry_d = np.radians(coords[1][4])  # Pitch angle (in radians)
    rz_d = np.radians(coords[1][5])  # Yaw angle (in radians)
    q_init = angles[1]
    joint_angles = ik((x_target, y_target, z_target, rx_d, ry_d, rz_d), q_init) # apply inverse kinematic to find computed angles
    calculated_coords = symbolic_forward_kinematics(joint_angles) # apply forward kinematics to find the computed coordinates with computed angles
    # Since the coordinates that we got only contains x y z... we then add the "recorded" rx ry rz
    end_effector_position = calculated_coords[:3, 3][0:3] + [coords[1][3], coords[1][4], coords[1][5]]
    # Send coordinates to robot
    mycobot.send_coords(end_effector_position, 10)
    print("Grapping States")
    sleep(5)
    mycobot.set_gripper_state(1,30) # Close gripper to grab the pen
    sleep(5)
    # View the current coordinate we at
    print("Coor:")
    for j in range(len(end_effector_position)):
        print("c" + str(j) + ": " + str(end_effector_position[j]))

    # Back to neutral
    print("Neutral States")
    mycobot.send_angles([0,0,0,0,0,0],30)

    # Find compute angles with inverse kinematics and then use forward kinematics to find the coordinates position at ending position (Spot to drop the marker)
    x_target = coords[2][0]
    y_target = coords[2][1]
    z_target = coords[2][2]
    rx_d = np.radians(coords[2][3])  # Roll angle (in radians)
    ry_d = np.radians(coords[2][4])  # Pitch angle (in radians)
    rz_d = np.radians(coords[2][5])  # Yaw angle (in radians)
    q_init = angles[2]
    joint_angles = ik((x_target, y_target, z_target, rx_d, ry_d, rz_d), q_init) # apply inverse kinematic to find computed angles
    calculated_coords = symbolic_forward_kinematics(joint_angles) # apply forward kinematics to find the computed coordinates with computed angles
    # Since the coordinates that we got only contains x y z... we then add the "recorded" rx ry rz
    end_effector_position = calculated_coords[:3, 3][0:3] + [coords[2][3], coords[2][4], coords[2][5]]
    # Send coordinates to robot
    mycobot.send_coords(end_effector_position, 30)
    print("Placing States")
    sleep(3)
    mycobot.set_gripper_state(0,30) # Open Gripper
    sleep(3)
    # View the current coordinate we at
    print("Coor:")
    for j in range(len(end_effector_position)):
        print("c" + str(j) + ": " + str(end_effector_position[j]))

    # Send arm back to neutral coordinates to end
    print("Neutral States")
    mycobot.send_angles([0,0,0,0,0,0],30)
    sleep(5)

# the "pickNplace.csv" contains our recorded angles and coordinates... the even line is angles(line 0, 2, 4) and odd line is coordinates(line 1, 3, 5)
# test_with_recorded_coords() # Uncomment this line to run test on our inverse kinematics + compare the printed result in terminal with file "pickNplace.csv"
# pickNPlace() # Uncomment this line to send robot to pick up the marker at the starting point and place is at ending point
