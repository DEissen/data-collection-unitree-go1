#!/usr/bin/python

import sys
import time
import os
import numpy as np

# get path to robot_interface lib from Unitree
file_dir = os.path.dirname(os.path.abspath(__file__))
path_to_append = os.path.join(file_dir, os.pardir, "lib/python/amd64") # last part might needed to be changed to "../lib/python/arm64" on other system

sys.path.append(path_to_append)
import robot_interface as sdk  # nopep8


def read_IMU_data():
    # initialize some variables
    runCounter = 0
    info_printed_once = False
    start_logging = False
    show_print = False
    # all data seems to be sampled with 50 Hz (9 - 10 equal measurements when sleep_in_seconds is 0.002)
    # => use at least 0.01 as sleep_in_seconds
    sleep_in_seconds = 0.01
    measurement_duration = 60
    log_interval = 10
    measurement_timestamp = 0

    HIGHLEVEL = 0xee
    LOWLEVEL = 0xff

    # lists to log the data
    mode_ar = []
    bodyHeight_ar = []
    footRaiseHeight_ar = []
    yawSpeed_ar = []
    footForce_ar = []
    velocity_ar = []
    gyroscope_ar = []
    accelerometer_ar = []
    rpy_ar = []
    temperature_ar = []

    # create measurement directory and get paths for the sensors
    mode_data_dir, bodyHeight_data_dir, footRaiseHeight_data_dir, yawSpeed_data_dir, footForce_data_dir, velocity_data_dir, gyroscope_data_dir, accelerometer_data_dir, rpy_data_dir, temperature_data_dir = create_measurement_folder()

    # set to True when LAN is used and to False if WLAN is used
    use_LAN = true

    if use_LAN:
        udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)
    else:
        udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.12.1", 8082)

    cmd = sdk.HighCmd()
    state = sdk.HighState()
    udp.InitCmdData(cmd)

    while True:
        time.sleep(sleep_in_seconds)

        udp.Recv()
        udp.GetRecv(state)

        # print general info once at the beginning after a short waiting time
        if not info_printed_once and runCounter > 1 / sleep_in_seconds:
            print(f"Header = {state.head}")
            print(f"levelFlag = {state.levelFlag}")
            print(f"frameReserve = {state.frameReserve}")
            print(f"SN = {state.SN}")
            print(f"version = {state.version}")
            print(f"bandwidth = {state.bandWidth}")
            print(f"crc = {state.crc}")
            info_printed_once = True
            measurement_timestamp = time.strftime("%H_%M_%S")
            start_logging = True

        # log everything that seems to be interesting every iteration after general info was logged once
        if start_logging:
            if show_print:
                print(f"logging for step {runCounter}")
                print(f"mode = {state.mode}")
                print(f"bodyHeight = {round(state.bodyHeight, 6)}")
                print(f"footRaiseHeight = {round(state.footRaiseHeight, 6)}")
                # yawSpeed = rotation speed of robot
                print(f"yawSpeed = {round(state.yawSpeed, 6)}")
                # meaning of footForce: 0 = vorne rechts; 1 = vorne links; 2 = hinten rechts; 3 = hinten links
                print(
                    f"footForce: {round(state.footForce[0], 6)}, {round(state.footForce[1], 6)}, {round(state.footForce[2], 6)}, {round(state.footForce[3], 6)}")
                # footForceEst is useless
                print(
                    f"footForceEst: {round(state.footForceEst[0], 6)}, {round(state.footForceEst[1], 6)}, {round(state.footForceEst[2], 6)}, {round(state.footForceEst[3], 6)}")
                # position does not seem to be useful
                print(
                    f"position: {round(state.position[0], 6)}, {round(state.position[1], 6)}, {round(state.position[2], 6)}")
                # meaning of velocity: 0 = vorwärts/ rückwärts; 1 = seitwärts; 2 = hoch/runter
                print(
                    f"velocity: {round(state.velocity[0], 6)}, {round(state.velocity[1], 6)}, {round(state.velocity[2], 6)}")
                print("IMU data:")
                # gyroscope: 0 = x; 1 = y; 2 = z
                print(
                    f"gyroscope: {round(state.imu.gyroscope[0], 6)}, {round(state.imu.gyroscope[1], 6)}, {round(state.imu.gyroscope[2], 6)}")
                # accelerometer: 0 = x; 1 = y; 2 = z
                print(
                    f"accelerometer: {round(state.imu.accelerometer[0], 6)}, {round(state.imu.accelerometer[1], 6)}, {round(state.imu.accelerometer[2], 6)}")
                # rpy = Euler angle: 0 = Roll; 1 = Pitch; 2 = Yaw
                print(
                    f"rpy: {round(state.imu.rpy[0], 6)}, {round(state.imu.rpy[1], 6)}, {round(state.imu.rpy[2], 6)}")
                print(f"temperature = {state.imu.temperature}")
                print("\n")

            mode_ar.append(state.mode)
            bodyHeight_ar.append(state.bodyHeight)
            footRaiseHeight_ar.append(state.footRaiseHeight)
            yawSpeed_ar.append(state.yawSpeed)
            footForce_ar.append(
                [state.footForce[0], state.footForce[1], state.footForce[2], state.footForce[3]])
            velocity_ar.append(
                [state.velocity[0], state.velocity[1], state.velocity[2]])
            gyroscope_ar.append(
                [state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]])
            accelerometer_ar.append(
                [state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2]])
            rpy_ar.append(
                [state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2]])
            temperature_ar.append(state.imu.temperature)

        # log in the specified interval
        if start_logging and ((runCounter * sleep_in_seconds) % log_interval == 0):
            # convert the lists to numpy arrays and save them
            mode_ar = np.asarray(mode_ar)
            np.savetxt(os.path.join(mode_data_dir,
                       f"{measurement_timestamp}.csv"), mode_ar, delimiter=";")

            bodyHeight_ar = np.asarray(bodyHeight_ar)
            np.savetxt(os.path.join(bodyHeight_data_dir, f"{measurement_timestamp}.csv"),
                       bodyHeight_ar, delimiter=";")

            footRaiseHeight_ar = np.asarray(footRaiseHeight_ar)
            np.savetxt(os.path.join(footRaiseHeight_data_dir, f"{measurement_timestamp}.csv"),
                       footRaiseHeight_ar, delimiter=";")

            yawSpeed_ar = np.asarray(yawSpeed_ar)
            np.savetxt(os.path.join(
                yawSpeed_data_dir, f"{measurement_timestamp}.csv"), yawSpeed_ar, delimiter=";")

            footForce_ar = np.asarray(footForce_ar)
            np.savetxt(os.path.join(footForce_data_dir, f"{measurement_timestamp}.csv"),
                       footForce_ar, delimiter=";")

            velocity_ar = np.asarray(velocity_ar)
            np.savetxt(os.path.join(
                velocity_data_dir, f"{measurement_timestamp}.csv"), velocity_ar, delimiter=";")

            gyroscope_ar = np.asarray(gyroscope_ar)
            np.savetxt(os.path.join(gyroscope_data_dir, f"{measurement_timestamp}.csv"),
                       gyroscope_ar, delimiter=";")

            accelerometer_ar = np.asarray(accelerometer_ar)
            np.savetxt(os.path.join(accelerometer_data_dir, f"{measurement_timestamp}.csv"),
                       accelerometer_ar, delimiter=";")

            rpy_ar = np.asarray(rpy_ar)
            np.savetxt(os.path.join(
                rpy_data_dir, f"{measurement_timestamp}.csv"), rpy_ar, delimiter=";")

            temperature_ar = np.asarray(temperature_ar)
            np.savetxt(os.path.join(temperature_data_dir, f"{measurement_timestamp}.csv"),
                       temperature_ar, delimiter=";")

            # reset lists for next measurement
            mode_ar = []
            bodyHeight_ar = []
            footRaiseHeight_ar = []
            yawSpeed_ar = []
            footForce_ar = []
            velocity_ar = []
            gyroscope_ar = []
            accelerometer_ar = []
            rpy_ar = []
            temperature_ar = []

            # get new measurement timestamp
            measurement_timestamp = time.strftime("%H_%M_%S")

        # end measurement after specified time
        if runCounter > measurement_duration / sleep_in_seconds:
            sys.exit(0)

        # prepare and send high level command to do nothing!
        cmd.mode = 0      # 0:idle, default stand      1:forced stand     2:walk continuously
        cmd.gaitType = 0
        cmd.speedLevel = 0
        cmd.footRaiseHeight = 0
        cmd.bodyHeight = 0
        cmd.euler = [0, 0, 0]
        cmd.velocity = [0, 0]
        cmd.yawSpeed = 0.0
        cmd.reserve = 0

        udp.SetSend(cmd)
        udp.Send()

        runCounter = runCounter + 1


def create_measurement_folder():
    # create folder for results (if it does not exist yet)
    measurement_timestamp = time.strftime("%H_%M_%S")

    mode_data_dir = f"./measurement_{measurement_timestamp}/mode"
    bodyHeight_data_dir = f"./measurement_{measurement_timestamp}/bodyHeight"
    footRaiseHeight_data_dir = f"./measurement_{measurement_timestamp}/footRaiseHeight"
    yawSpeed_data_dir = f"./measurement_{measurement_timestamp}/yawSpeed"
    footForce_data_dir = f"./measurement_{measurement_timestamp}/footForce"
    velocity_data_dir = f"./measurement_{measurement_timestamp}/velocity"
    gyroscope_data_dir = f"./measurement_{measurement_timestamp}/gyroscope"
    accelerometer_data_dir = f"./measurement_{measurement_timestamp}/accelerometer"
    rpy_data_dir = f"./measurement_{measurement_timestamp}/rpy"
    temperature_data_dir = f"./measurement_{measurement_timestamp}/temperature"

    os.makedirs(mode_data_dir, exist_ok=True)
    os.makedirs(bodyHeight_data_dir, exist_ok=True)
    os.makedirs(footRaiseHeight_data_dir, exist_ok=True)
    os.makedirs(yawSpeed_data_dir, exist_ok=True)
    os.makedirs(footForce_data_dir, exist_ok=True)
    os.makedirs(velocity_data_dir, exist_ok=True)
    os.makedirs(gyroscope_data_dir, exist_ok=True)
    os.makedirs(accelerometer_data_dir, exist_ok=True)
    os.makedirs(rpy_data_dir, exist_ok=True)
    os.makedirs(temperature_data_dir, exist_ok=True)

    return mode_data_dir, bodyHeight_data_dir, footRaiseHeight_data_dir, yawSpeed_data_dir, footForce_data_dir, velocity_data_dir, gyroscope_data_dir, accelerometer_data_dir, rpy_data_dir, temperature_data_dir


if __name__ == '__main__':
    read_IMU_data()
