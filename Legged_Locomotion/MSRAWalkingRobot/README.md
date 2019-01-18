# Walking Robot Simulation
### Copyright 2017-2019 The MathWorks, Inc.

## ORIGINAL EXAMPLE FILES
### 1. startupDemo
This will set up the search path and open the main model, walkingRobot.slx

### 2. optimizeRobotMotion
This is the main optimization script, which has several options.
   
#### a. parallelFlag [true|false]
Use Parallel Computing Toolbox or MATLAB Distributed Computing Server to speed up optimization.
This requires that you have already set up a default parallel computing cluster.

#### b. accelFlag [true|false]
Run the simulations in Accelerator mode.
This will turn off visualization with the Mechanics Explorer, but will run faster.

#### c. actuatorType [1|2|3]
Different fidelity actuators for simulation. Suggest running optimization with actuatorType = 2.
* 1: Ideal motion actuation
* 2: Torque actuation with PID control
* 3: Electric motor actuation with average-model PWM, H-Bridge, and PID control

### 3. compareActuatorTypes 
Compares simulation results for a trajectory across different actuator types.

---

## INVERSE KINEMATICS EXAMPLE FILES
The `Robot/InverseKinematics' folder contains examples for leg inverse kinematics.

### 1. calculateInvKin.mlx
This Live Script is used to derive an analytical expression to compute leg 
inverse kinematics in the longitudinal/sagittal direction. It generates a MATLAB
function named `legInvKin.m`.

### 2. animateFootGait.m
Script that animates the walking gait according to the specified parameters, and 
tests the inverse kinematics calculation above.

### 3. walkingRobotFootPlace.slx
Simulink model showing how to create virtual joints at the foot to directly
actuate the motion of each foot according to a specified stepping plan.

### 4. walkingRobot InvKin.slx
Simulink model showing how to integrate the inverse kinematics and step planners
from the previous files. 

---

## 3D WALKING EXAMPLE FILES
The `Robot/3D` folder contains newer examples for 3D walking control.

### 1. walkingRobot3DPath.slx
Simulink model showing the full 3D control. This robot has different parameters 
compared to the original example, to show the robustness of this control with a 
more realistic robot (thinner torso, smaller feet, less world damping, etc.). 
This uses the `robotParametersZMP.m` data file.

### 2. plotWalkingTraj.m
MATLAB script that takes logged data from the previous model and animates the 
trajectory for the torso and the feet.

### 3. walkingRobot3DMPC.slx
Simulink model showing the full 3D control with a trajectory generated using
Model Predictive Control (MPC).

---

## Multiphysics and Contact Libraries
For convenience, local copies of the Simscape Multibody Multiphysics Library and
Simscape Multibody Contact Forces Library have been included with this submission. 
If you would like to install the latest version of these libraries, you can find
them from the Add-On Explorer, or on the File Exchange
* Simscape Multibody Multiphysics Library: https://www.mathworks.com/matlabcentral/fileexchange/37636-simscape-multibody-multiphysics-library
* Simscape Multibody Contact Forces Library: https://www.mathworks.com/matlabcentral/fileexchange/47417-simscape-multibody-contact-forces-library

