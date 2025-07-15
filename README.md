# Robot 6 Axis - Cartesian Motion Control

## Description

This project implements the control of a stationary 6-axis robot arm, featuring forward and inverse kinematics, smooth trajectory generation, and simple 3D movement visualization.

The program accepts user commands for the desired Cartesian position and orientation (x, y, z, roll, pitch, yaw in degrees), checks if the target is reachable within joint limits, generates an interpolated trajectory, and executes the movement while displaying intermediate positions and joint angles.

---

## Features Implemented

- Robot arm structure defined with motion limits for each joint.
- Forward and inverse kinematics calculations using symbolic methods with SymPy.
- User input for specifying the desired end-effector pose.
- Joint limit validation to ensure reachable targets.
- Smooth trajectory generation using linear interpolation.
- Movement visualization using 3D matplotlib plots.
- Unit tests for core kinematics functions and joint limit checks.

---

## Requirements

- Python 3.12.8 (tested with this version)
- Python libraries:
  - sympy==1.13.1
  - matplotlib==3.10.0
- unittest (built-in Python module, no installation needed)

## How to run the program

1. Clone or download this repository.

2. Navigate to the project folder:
    
    ```bash
    cd 6dof_cartesian_motion
    ```

3. Install dependencies (if not already installed):

    ```bash
    pip install sympy==1.13.1 matplotlib==3.10.0
    ```

4. Run the main program:

    ```bash
    python main.py
    ```

5. When prompted, enter the desired end-effector pose in the format:

    x y z roll pitch yaw

   - `x, y, z` in meters  
   - `roll, pitch, yaw` in degrees

   Example input:

    1.0 1.0 1.0 0 45 90

6. Observe the robot arm moving smoothly to the target pose, with printed joint angles and a 3D visualization.

7. To exit, type `sair` or `exit`.


