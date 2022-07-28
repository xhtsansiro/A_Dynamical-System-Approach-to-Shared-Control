# Robot_Control

## Introduction

This ROS package is a modified version of the [Project Laboratory Human Centered Robotics](https://github.com/BernardoBrogi/Project-Laboratory). Based on that, the learning-based shared control approach is implemented, providing haptic guidance to the human operator when the human moves the robot with the haptic device. 


### Structure of the package

* The folder [gazebo_model](gazebo_model/) contains several models used in teleoperation scenario in Gazebo.

* All the other folders are directly taken from [Project Laboratory Human Centered Robotics](https://github.com/BernardoBrogi/Project-Laboratory).

* To implement shared control approach, several functionalities are implemented and added in the package [lwr_simple_example](kuka-lwr-ros-examples/lwr_task_examples/lwr_simple_example). The source files of implemented actions are in the folder [simple_actions](kuka-lwr-ros-examples/lwr_task_examples/lwr_simple_example/src/simple_actions)(kuka-lwr-ros-examples/lwr_task_examples/lwr_simple_example/src/simple_actions).


* The folder [data](data/) contains the Gaussian Process (GP) dataset generated from demonstrations, which is used for Learning from demonstration (LfD). Besides, the logging data of the experiment is saved in this folder. 
    - Subfolder [backup_GP_dataset](data/backup_GP_dataset/): backup of GP dataset.
    - Subfolder [incremental_learning](data/incremental_learning/): logging data of the new demonstration used for incremental learning.
    - Subfolder [record](data/record/): logging data of the original demonstration.
    - Subfolder [shared_control](data/shared_control/): logging data of the trial executed with the guidance from controller.
    - Subfolder [stiffness_plot](data/stiffness_plot/): logging data of the desired stiffness along the reference path.
    - Subfolder [user_study](data/user_study/): logging data of the user study.
    - [position](data/pos_train.txt) and [velocity](data/vel_train.txt): GP dataset for learning a DS, incremental learning is done by simply expanding this dataset.


## How to use

This package runs perfectly in ROS melodic (full desktop version) and GAZEBO 9.

### Build the package

* download this package into the workspace `catkin_ws` (user-defined, any other workspace is fine). 
* cd to the directory `catkin_ws`
* type the command in the terminal 
```
$ catkin_make --pkg lwr_simple_example
```
In case of any additional added implemented functionalities, remember to add the directory of its source file and header file in [CMakeList](kuka-lwr-ros-examples/lwr_task_examples/lwr_simple_example/CMakeLists.txt), then rebuild the package by repeating the last step.


### Launch the simulation

After building the package successfully, the simulation can be started. In order to start the simulation, open three different terminal windows.

First, execute the command in all terminal windows.
```
$ source devel/setup.bash
```
This should be done every time after building the package.

After that, execute three commands in three different terminal windows (one command in one terminal window)
```
$ roslaunch lwr_simple_example sim.launch
```

which will setup the simulation environment. This command should be firstly executed. 

```
$ roslaunch lwr_simple_example client.launch
```
which will run the client.

```
$ roslaunch lwr_simple_example console.launch
```

This is the console from which one can command the robot. Execute the command in the terminal

```
$ Cmd> command
```
Command is a placeholder and possible options to execute in using this approach are as follows:
* `go_home`:    move the robot to its homing position.
* `Record`:     teleoperate the robot with the haptic device in free mode, normally used for collecting the first demonstration.
* `Guidance`:   execute target-reaching task under the guidance from VSDS controller.
* `Restart`:    set the haptic device to free mode, the user can move the robot to any new starting positions with the haptic device.
* `Compare`:    start the user-study, where different controllers are compared in a target-reaching task.
* `Fix_point`:    Move the robot to a pre-defined position.

Notice: the robot means the end-effector of the robot here.

The environment of the simulation in Gazebo is shown as follow ![The simulation environment](https://github.com/xhtsansiro/Shared_Control/blob/main/pics/environment.png)


### Task execution with shared control

#### Haptic Device setup

The haptic device is the [omega.3](https://www.forcedimension.com/images/doc/specsheet_-_omega3.pdf). You may need to adjust the path to the libraries of the haptic device in the CMakelist.txt.
You may also need to give the permission to the port where the haptic device is inserted. To do this type in a terminal

```
$ ls -l /dev/bus/usb/00*
```
Which will give the list of devices connected to the port. Check which is the port connected to the haptic device.

Suppose this corresponds to your device
```
/dev/bus/usb/003
crw-rw-r-- 1 root root 189, 263  1월 10 15:42 008
```
Then type this command to give permisssion
```
sudo chmod o+w /dev/bus/usb/003/008
```


#### Collection of the first demonstration

Start three terminal windows and launch the corresponding launch files as mentioned above.

First, execute the command 
```
$ Cmd> go_home
```

Then, execute the command 
```
$ Cmd> Record
```

The user is required to do the first demonstration, and the data is saved in [record](data/record/). The first demonstration is then processed offline by Matlab script [preprocessing](https://github.com/xhtsansiro/Shared_Control/Data_Analysis/01_Implementation/preprocessing.m). Afterwards, save the position and velocity data in folder [data](data/)



#### Execution with shared control

Start three terminal windows and launch the corresponding launch files as mentioned above.

execute the command 
```
$ Cmd> go_home
```

Then execute the command 
```
$ Cmd> Guidance
```
where the user receives the haptic guidance from VSDS controller and executes the task.After reaching the target, execute the following command to move the robot to any other starting positions.
```
$ Cmd> Restart
```

Afterwards, execute the following command, the task is executed with the guidance. 
```
$ Cmd> Guidance
```

When the incremental learning is necessary, the user escapes the guidance during task execution, goes back to the initial position approximately(press a key confirms the move back), demonstrates a new path, starts incremental learning (press a key confirms the learning). 

After incremental learning, execute the command to go back to the initial position,
```
$ Cmd> Fix_point
where the user can set the initial position based on tasks by modifying the position setting in the file [fix_point](kuka-lwr-ros-examples/lwr_task_examples/lwr_simple_example/src/simple_actions/Fix_point.cpp).

```
Move the robot to the same starting position, then execute the command
```
$ Cmd> Guidance
```

#### User study

Start three terminal windows and launch the corresponding launch files as mentioned above.

execute the command 
```
$ Cmd> go_home
```

Then execute the command 
```
$ Cmd> Compare
```

#### Adapt scenario in Gazebo

When it is necessary to adapt the Gazebo scenario, such as adding new obstacles, steps are following:
* First, create a new gazebo model, save it in folder [gazebo_model](gazebo_model/)
* Then, adapt the world file [simple_environment](kuka-lwr-ros-examples/lwr_robot_examples/kuka-lwr-single/lwr_robot/single_lwr_robot/worlds/simple_environment.world), such as adding the new obstacle into the world file.
* The current world file is the standard scenario.
* The scenario where a new obstacle exists is set in file [environment_new_obstacle](kuka-lwr-ros-examples/lwr_robot_examples/kuka-lwr-single/lwr_robot/single_lwr_robot/worlds/simple_environment_with_new_obstacle.world)

If the user wants to adapt the camera view, please change the setting of 'gzclient_camera' in [simple_environment](kuka-lwr-ros-examples/lwr_robot_examples/kuka-lwr-single/lwr_robot/single_lwr_robot/worlds/simple_environment.world).
* The current camera view is for the experiment validation.
* The camera view setting of the user study is in file [environment_user_study](kuka-lwr-ros-examples/lwr_robot_examples/kuka-lwr-single/lwr_robot/single_lwr_robot/worlds/simple_environment_user_study.world).



### Notice:
* During the experiment validation, deactivate the safety constraints in file [joint_controller](kuka-lwr-ros/kuka_lwr/lwr_controllers/src/joint_controllers.cpp) line 264 - line 274. This avoids that the robot must go home position every time after finishing the task, such that directly going to other starting position for a new task is possible. 
However, it is not a clean solution. A clean solution can be obtained by improving the implementation in [contact_safety](kuka-lwr-ros/kuka_lwr/lwr_controllers/src/utils/contact_safety.cpp)

* The safety constraints is activated during the user study.

* Remember to adapt the absolute paths used in the source files of [lwr_simple_example](kuka-lwr-ros-examples/lwr_task_examples/lwr_simple_example/src/simple_actions) and in the head files of [lwr_simple_example](kuka-lwr-ros-examples/lwr_task_examples/lwr_simple_example/include/simple_actions)



