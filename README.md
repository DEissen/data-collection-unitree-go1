# Unitree-Go1-Edu repository
This repository contains the code to derive data from all cameras and the IMU (including foot force, ...) of the Unitree Go1 robot dog. \
It consists of four modules where three contain the code for specific sensors while the fourth module contains some custom util functions for this project. On the top level of the repository there are two shell scripts and one python file to record in parallel images with all cameras and collect the data from the IMU of the Go1 including automated copying of the images to the local PC. For more details about how to start a measurement and about all modules see the following chapters. 

# How to use the code to create a measurement with the Go1?
This section explains how to use the code in this repo for creating new measurements. Further details about the code can be found in later chapters.
### Prerequisites
1. Code must be executed on Linux machine (for Go1 libraries), using WSL with Windows is also possible
2. LAN connection between Go1 and your PC
3. You can ping all Nano's of Go1 (IP-addresses 192.168.123.13 to .15)
4. Custom camera programs must be built and available on each Nano at location:
    - /home/unitree/Unitree/sdk/UnitreeCameraSdk/bins/getFrameXXXCamera (XXX = One for Nano 15 / XXX = Two for Nanos 13 and 14)
### Preparation
1. Update the local time of each Nano. 
    - You have to modify and execute the shell scripts set_time.sh which can be found here:
        - In the repo:
            - .\camera\src_Nanos\set_time.sh
        - On each Nano:
            - \home\unitree\Documents\set_time.sh
    - It's recommended to update the files directly on the Nano's, that way you don't have to copy the files there afterwards.
    - After modifying the user and the IP-address for your PC, you only need to execute the script on each Nano.
2. Kill running default camera processes to enable custom program to run.
    - Execute the shell script killCameraProcesses.sh on each Nano, which can be found here:
        - \home\unitree\Documents\killCameraProcesses.sh
### Start new measurement
1. Prepare ./camera/src_local_PC/copy_images.py
    - Potentially modify the variable "destination" which contains the path to the dir, where the measurement results will be stored.
2. Prepare ./main.py
    - Potentially modify the variable "measurement_base_path" which contains the path to the dir, where the measurement results will be stored.
    - **Hint:** To get the images and the IMU measurements stored directly in the same directory, "measurement_base_path" should be "destination" + "/data" (copy_images.sh will copy the whole /data dir from the Nanos, thus the additional directory is needed)
3. Execute the shell script ./main.sh.
    - You will be asked first to select the floor type which this measurement is done on. Provide this information by typing the appropriate number and confirm by pressing Enter.
    - **Important**: Stop the measurement by typing "stop" in the command line and confirm by pressing Enter. Only this way the measurement will be stopped properly and all data can be stored by all programs.

# read_IMU_data
NOTE: This module contains the dynamic libraries from [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk) repository for the robot_interface module which is needed to derive the data from the IMU. Thus this module also contains the LICENSE file of the  unitree_legged_sdk repository.


# Authors
- Dominik Eißen (st177975@stud.uni-stuttgart.de)