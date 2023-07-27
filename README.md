# Unitree-Go1-Edu repository
This repository contains the code to derive data from all cameras and the IMU of the Unitree Go1 robot dog. \
It consists of four modules where three contain the code for specific sensors while the fourth module contains some custom utils for this project. On the top level of the repository there are two shell scripts and one python file to record in parallel images with all cameras and collect the data from the IMU of the Go1 including automated copying of the images to the local PC. For more details about each file and the modules see the following chapters (TODO). 

# read_IMU_data
NOTE: This module contains the dynamic libraries from [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk) repository for the robot_interface module which is needed to derive the data from the IMU. Thus this module also contains the LICENSE file of the  unitree_legged_sdk repository.


# Authors
- Dominik Eißen (st177975@stud.uni-stuttgart.de)