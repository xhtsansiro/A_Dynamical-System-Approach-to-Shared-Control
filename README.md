# A Dynamical System Approach to Shared Control

## Introduction

This project proposes a learning-based shared control approach which uses first order time invariant Dynamical Systems (DS) as motion generator and Variable Stiffness Dynamical Systems (VSDS) Controller as guidance generator. The proposed approach is considered for a teleoperation scenario, where the user interacts with the master device, controlling the remote robot to execute a specific task jointly. ![The overall architecture of the shared control approach](https://github.com/xhtsansiro/Shared_Control/blob/main/pics/Architecture_VSDS.png).

The proposed approach is validated in a teleoperation scenario, where a target-reaching task is executed. The 3-DOF omega.3 haptic device is the master device, and 7-DOF KUKA LWR in Gazebo is the remote robot. The target to reach is the upper surface of the pink object placed in a box. ![The setting of teleoperation](https://github.com/xhtsansiro/Shared_Control/blob/main/pics/teleoperation.png).

The approach is validated by normal execution and incremental learning. Besides, user-study is conducted to compare the generated haptic guidances among different controllers. 
* VSDS Controller 
* Flow Controller 
* Open-loop Impedance Controller 
* Free Mode, without guidance.

## Structure

* The package [Robot_Control](Robot_Control/): A ROS package, implements the proposed approach, settings of the , and so on.

* The package [Data_Analysis](Data_Analysis/): Analyze the data from experiment and user study, visualize the results.



