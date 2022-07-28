# Data Analysis 

## Introduction


This Matlab project visualizes the proposed shared control approach, and analyzes the data of validation experiment and the data of user study. 

### Scripts
* `preprocessing.m`: preprocess the original demonstration, generate Gaussian Process(GP) dataset.
* `gp_prediction.m`: Implement Gaussian Process Regression and visualize its performance.
* `gp_vsds.m`: Visualize the construction of Variable Stiffness Dynamical Systems (VSDS), and check the performance of VSDS used in a simple second order DS.
* `plot_all.m`: Generate all necessary plots to visualize results of validation experiment.  
* `analyze_userstudy.m`: Generate all necessary plots to visualize results of the user study.
* [video_generation.m](video_generation.m): Generate videos of the robot motion. 

### Data 
* Data of validation experiment are saved in folder [data](data/)
* Data of user study are saved in folder [user_study](user_study/)


## Usage

Find the corresponding Matlab script, and execute it. 