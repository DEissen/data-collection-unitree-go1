# Unitree-Go1-Edu repository
This repository contains the code to derive data from all cameras and the IMU (including foot force, ...) of the Unitree Go1 robot dog. \
It consists of four modules where three contain the code for specific sensors while the fourth module contains some custom util functions for this project. On the top level of the repository there are two shell scripts and one python file to record in parallel images with all cameras and collect the data from the IMU (including all the other timeseries sensors, e.g. foot force) of the Go1 including automated copying of the images to the local PC. For more details about how to start a measurement and about all modules see the following chapters. 

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
    - **Note:** This is only possible on a native Linux machine, as SSH connection to WSL is only possible with some further changes in your Windows system!
2. **[Optional]** Kill running default camera processes to enable custom program to run.
    - Execute the shell script killCameraProcesses.sh on each Nano, which can be found here:
        - \home\unitree\Documents\killCameraProcesses.sh
    - This is optional, as this always happens when starting the measurement using main.py or main.sh
### Start new measurement
1. Prepare ./camera/src_local_PC/copy_images.py
    - Potentially modify the variable "destination" which contains the path to the dir, where the measurement results will be stored.
2. Prepare ./main.py
    - Potentially modify the variable "measurement_base_path" which contains the path to the dir, where the measurement results will be stored.
    - **Hint:** To get the images and the IMU measurements stored directly in the same directory, "measurement_base_path" should be "destination" + "/data" (copy_images.sh will copy the whole /data dir from the Nanos, thus the additional directory is needed)
3. Execute the shell script ./main.sh.
    - You will be asked first to select the floor type which this measurement is done on. Provide this information by typing the appropriate number and confirm by pressing Enter.
    - **Important**: Stop the measurement by typing "stop" in the command line and confirm by pressing Enter. Only this way the measurement will be stopped properly and all data can be stored by all programs.

### Result of a measurement
If you have done a measurement using the *main.sh* script following all hints from the above chapter, you should find a **data/** directory containing all measurements in the following structure:
- **data/**
    - **measurement_DD_MM__HH_XX/**
        - **XXXCamLeft/**: Folder containing all images (stored as .jpg files) of the right camera from stereo camera XXX
        - **XXXCamRight/**: Folder containing all images (stored as .jpg files) of the right camera from stereo camera XXX
        - **YYY/**: Folder containing timeseries data in CSV files for the sensor YYY (e.g. footForce, gyroscope, ...)
        - *info.json*: JSON file containing all important information for data preparation (label, time diffs, ...)
    - **measurement_DD_MM__HH_XY/**
        - ...

# Folder structure and module descriptions
This section contains a brief overview about all files in the repository. The code is structured in four modules/subfolder which contain code for different purposes.
- **camera/** \
This module contains all code and script files related to the image capturing.
    - **src_local_PC/** \
    This module contains code which has to be executed on the local PC.
        - *copy_images.py:* Functions to copy all images of a measurement via scp.
        - *start_measurement.py:* Functions to start camera capturing programs at the Go1's Nano's including functions to determine time difference between local PC and the Nano's. \
        *Note:* To start all programs in parallel, Threads have to be used for which this file also contains example code.
    - **src_Nanos/** \
    This module contains code which has to be executed on the Nano's of the Go1. The C++ files must be built on the Nano's directly!
        - *getFrameOneCamera.cc:* C++ program for camera capturing (including creation of folder structure for results, ...) with one camera.
        - *getFrameTwoCameras.cc:* C++ program for camera capturing (including creation of folder structure for results, ...) with two cameras.
        - *killCameraProcesses.sh:* Shell script to terminate all camera related processes on a Nano. Necessary, as only one program can use a camera at once.
        - *set_time.sh:* Shell script to set the time to the time of another PC by using a SSH connection. Must be done as the time on a Nano can't be changed permanently
- **custom_utils/** \
This module contains some custom utility functions used in the repository.
    - *utils.py:* Utility functions to get floor type via command line and to save dicts as JSON file.
- **read_IMU_data/** \
This module contains all files needed to read timeseries sensors data (IMU, foot force, ...) which is summarized called as IMU data here.
    - **lib/**: Contains dynamic libraries from [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk) repository for the robot_interface module. Structure won't be further explained.
    - **scripts/**
        - *example_walk.py:* Example from unitree_legged_sdk repository as a reference of how to use the robot_interface module.
        - *read_IMU.py:* ReadImuDataGo1() Thread class to retrieve IMU data (including all other timeseries sensors) by using the robot_interface module.
    - *LICENSE*: License file of the [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk) repository for the robot_interface module which is needed to derive the data from the IMU and the other timeseries sensors.
- **read_with_ROS/** \
This module contains a file to retrieved data from the Go1 via a ROS Node. \
*NOTE:* Currently not used by the main program.
    - *pointCloud_Subscriber_Node.py:* Functions for ROS Subscriber Nodes to read PointClouds.
- *copy_images.sh:* Shell script to execute *./camera/src_local_PC/copy_images.py* from top level of repository.
- *example_data.png*: Image showhing example data for README.md
- *main.py:* Code to start a measurement of IMU data and camera images in parallel including determination of time diff and storage of label information.
- *main.sh:* Shell script to execute *main.py* and *./camera/src_local_PC/copy_images.py* subsequently, to execute complete measurement including copying the images from the Nano's.
- *README.md*: The file you are reading right now :)
# Supported sensors and example data
Here is a full list of the sensors for which data will be recorded after proper preparation of the Go1:
- accelerometer
- BellyCamLeft
- BellyCamRight
- bodyHeight
- ChinCamLeft
- ChinCamRight
- footForce
- gyroscope
- HeadCamLeft
- HeadCamRight
- LeftCamLeft
- LeftCamRight
- mode
- RightCamLeft
- RightCamRight
- rpy
- velocity
- yawSpeed

Here is an example of the camera images and the corresponding accelerometer for a measurement:
![Example data](./example_data.png)

# Authors
- Dominik Eißen (st177975@stud.uni-stuttgart.de)