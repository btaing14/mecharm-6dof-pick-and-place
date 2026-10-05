# 6-DOF Robotic Arm Kinematics & Pick-and-Place

Python control software for a **MechArm 270 Pi** 6-DOF robotic arm. It covers drag-teach position recording, a custom forward/inverse kinematics model, and two manipulation tasks: precision peg insertion, and taking apart and rebuilding a block tower alongside a human operator.

Built as a three-person team project for EE 347 (Robotics & Controls) at the University of Washington, Autumn 2025.

## Demo

| Challenge 1: Peg insertion | Challenge 2: Tower rebuild |
|---|---|
| [▶ Watch video](media/challenge1_peg_insertion.mp4) | [▶ Watch video](media/challenge2_tower_rebuild.mp4) |
| Picks three pegs from a fixed loading zone and inserts each into its color-coded slot on a slotted bench. | Removes the 1st, 3rd and 5th blocks from a 5-block tower and stacks them into a new tower while a human handles the rest. |

## What's in this repo

```
Lab2/  mecharm_control_group_08.py      Drag teaching, recording joint angles and end-effector poses to CSV
       data.csv                         Recorded positions
Lab3/  mecharm_INVK_advanced_group_08.py DH-parameter forward kinematics + numerical inverse kinematics
       pickNplace.csv                   Recorded positions for a marker pick-and-place test
Lab4/  challenge_peg_group_08.py        Challenge 1: peg insertion
       challenge_tower_group_08.py      Challenge 2: tower deconstruct/rebuild (finite-state machine)
       peg.csv, block.csv               Tuned target poses for each challenge
media/                                  Demo videos
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

## Known limitations & next steps

These are issues found when reviewing the code after the course:

- **The arm's built-in solver does the final joint solve.** `getCoordinates()` computes IK, runs it back through FK, and sends the resulting *Cartesian* pose to `send_coords()`. The robot firmware then works out its own joint angles. So the custom model refines and checks targets but doesn't command the joints. *Next step:* send the IK solution directly with `send_angles()`.
- **Yaw extraction bug:** in `transf_to_pose()`, yaw uses `arctan2(T[1][0], T[0][1])`. It should be `arctan2(T[1][0], T[0][0])`, so the orientation part of the IK error is wrong.
- **Degree/radian mismatch:** the FK model works in radians, but the IK initial guess comes from `get_angles()` (degrees), and the bounds `(-180, 180)` are treated as radians. It worked in practice because only the position output was used, but the seed isn't really the arm's current pose.
- **No IK failure handling:** if the solver doesn't converge, `ik()` returns `None` and the next call raises an error.
- **Open-loop timing:** moves are sequenced with fixed `sleep()` delays rather than checking whether the arm has arrived. Polling the arm's position, or adding camera feedback, would speed things up and improve accuracy.

The code is kept as it ran in the demo videos.

## Running it

Requires a MechArm 270 Pi with its Raspberry Pi and Python 3.

```bash
pip install pymycobot numpy sympy scipy
cd Lab4
python3 challenge_peg_group_08.py      # Challenge 1
python3 challenge_tower_group_08.py    # Challenge 2
```

Run from inside `Lab4/` so the CSV paths resolve. The target poses in `peg.csv` and `block.csv` are specific to our bench setup and will need re-teaching on a different setup (uncomment `Read10Position()` to drag-teach).

## Tech stack

Python · NumPy · SymPy · SciPy · pymycobot · Raspberry Pi

## Team

Steven Gong, Nicholas Leung, Bobby Taing
