# 6-DOF Robotic Arm Kinematics & Pick-and-Place

Python control software for a **MechArm 270 Pi** 6-DOF robotic arm. It covers drag-teach position recording, a custom forward/inverse kinematics model, and two manipulation tasks: precision peg insertion, and taking apart and rebuilding a block tower alongside a human operator.

Built as a three-person team project for EE 347 (Robotics & Controls) at the University of Washington, Autumn 2025.

## Demo

**Challenge 1: Peg insertion**

https://github.com/user-attachments/assets/2ec03f14-8a59-47b5-bc81-ef12e6fd41f6

**Challenge 2: Tower rebuild**

https://github.com/user-attachments/assets/a69bf12b-cfcb-447e-b5ca-fc2435501352

## What's in this repo

```
Lab2/  mecharm_control_group_08.py      Drag teaching, recording joint angles and end-effector poses to CSV
       data.csv                         Recorded positions
Lab3/  mecharm_INVK_advanced_group_08.py DH-parameter forward kinematics + numerical inverse kinematics
       pickNplace.csv                   Recorded positions for a marker pick-and-place test
Lab4/  challenge_peg_group_08.py        Challenge 1: peg insertion
       challenge_tower_group_08.py      Challenge 2: tower deconstruct/rebuild (finite-state machine)
       peg.csv, block.csv               Tuned target poses for each challenge
```

## How it works

**1. Drag teaching (Lab 2).** The servos are released so the arm can be moved by hand to each target. `Read10Position()` records the joint angles and end-effector pose at each stop. The CSV files alternate lines: joint angles, then the matching `[x, y, z, rx, ry, rz]` pose.

**2. Kinematics model (Lab 3).**
- **Forward kinematics:** builds the base-to-tool transform `T06` symbolically in SymPy from the arm's Denavit-Hartenberg table, then compiles it to a fast NumPy function with `lambdify`.
- **Inverse kinematics:** solves for joint angles by minimizing the 6-DOF pose error (position + roll/pitch/yaw) between FK and the target pose, using SciPy's `least_squares` (trust-region reflective, bounded).
- `getCoordinates()` runs each recorded target through IK, then FK, and sends the resulting pose to the arm.

**3. Motion planning (Lab 4).**
- **Via points:** every pick or place first moves to a hover point above the target, then moves straight down. This stopped the arm from knocking over pegs or the tower on the way in.
- **Neutral-pose reset:** the arm returns to all-zero joint angles between operations as a known reference, which made runs more repeatable.
- **Speed control:** travel moves are capped at 30% speed, and final insertion moves run at 10%.
- **Finite-state machine (Challenge 2):** alternates between a *pick* state and a *drop* state. Each state runs a fixed sequence of via point, target, gripper action, via point, then neutral. This made it easier to see which stage failed and fix it.

## Results

- **Challenge 1:** 3/3 pegs placed in about 2.5 minutes, with 2 allowed human interventions to fully seat the last two pegs.
- **Challenge 2:** tower taken apart and rebuilt along a collision-free path. Placement accuracy limited how straight the new tower was, and 2 interventions were used to straighten it.

## Problems solved on hardware

- **Gripper stalls and crashes:** fully closing the gripper on a peg strained the servo and sometimes crashed the arm. Switching to a partial grip with `set_gripper_value()` held pegs securely and stopped the crashes.
- **Gripper not opening:** release commands were sometimes ignored, so the code re-sends "open" at the via point after each placement.
- **Drag-teach inaccuracy:** recorded poses didn't reproduce exactly, so targets were tuned one axis at a time in the CSVs. The remaining slots were placed as X/Y offsets from the first.
- **Reach limit:** the red slot was too close to the bench edge for the end effector, so the placement order was changed to yellow → light green → orange.

  **Team:** Steven Gong, Nicholas Leung, Bobby Taing
