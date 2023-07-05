#!/usr/bin/python

import sys
import time
import math
import socket
import numpy as np

sys.path.append('../lib/python/amd64')
import robot_interface as sdk


if __name__ == '__main__':
    # initialize some variables
    info_printed_once = False
    start_logging = False
    show_print = False
    sleep_in_seconds = 0.002

    HIGHLEVEL = 0xee
    LOWLEVEL  = 0xff

    # arrays to log the stuff
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

    # check own IP adress to determine IP adress for udp communication
    hostname = socket.gethostname()
    ip_addr = socket.gethostbyname(hostname)

    if "192.168.123" in ip_addr:
        udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)
    else:
        udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.12.1", 8082)

    cmd = sdk.HighCmd()
    state = sdk.HighState()
    udp.InitCmdData(cmd)

    motiontime = 0
    while True:
        time.sleep(sleep_in_seconds)
        motiontime = motiontime + 1

        udp.Recv()
        udp.GetRecv(state)
        
        # print general info once at the beginning after a short waiting time
        if not info_printed_once and motiontime > 1 / sleep_in_seconds:
            print(f"Header = {state.head}")
            print(f"levelFlag = {state.levelFlag}")
            print(f"frameReserve = {state.frameReserve}")
            print(f"SN = {state.SN}")
            print(f"version = {state.version}")
            print(f"bandwidth = {state.bandWidth}")
            print(f"crc = {state.crc}")
            info_printed_once = True
            start_logging = True

        # log everything that seems to be interesting every iteration after general info was logged once
        if start_logging:
            if show_print:
                print(f"logging for step {motiontime}")
                print(f"mode = {state.mode}")
                print(f"bodyHeight = {round(state.bodyHeight, 6)}")
                print(f"footRaiseHeight = {round(state.footRaiseHeight, 6)}")
                # yawSpeed = rotation speed of robot
                print(f"yawSpeed = {round(state.yawSpeed, 6)}")
                # meaning of footForce: 0 = vorne rechts; 1 = vorne links; 2 = hinten rechts; 3 = hinten links
                print(f"footForce: {round(state.footForce[0], 6)}, {round(state.footForce[1], 6)}, {round(state.footForce[2], 6)}, {round(state.footForce[3], 6)}")
                # footForceEst is useless
                print(f"footForceEst: {round(state.footForceEst[0], 6)}, {round(state.footForceEst[1], 6)}, {round(state.footForceEst[2], 6)}, {round(state.footForceEst[3], 6)}")
                # position does not seem to be useful
                print(f"position: {round(state.position[0], 6)}, {round(state.position[1], 6)}, {round(state.position[2], 6)}")
                # meaning of velocity: 0 = vorwärts/ rückwärts; 1 = seitwärts; 2 = hoch/runter
                print(f"velocity: {round(state.velocity[0], 6)}, {round(state.velocity[1], 6)}, {round(state.velocity[2], 6)}")
                print("IMU data:")
                # gyroscope: 0 = x; 1 = y; 2 = z
                print(f"gyroscope: {round(state.imu.gyroscope[0], 6)}, {round(state.imu.gyroscope[1], 6)}, {round(state.imu.gyroscope[2], 6)}")
                # accelerometer: 0 = x; 1 = y; 2 = z
                print(f"accelerometer: {round(state.imu.accelerometer[0], 6)}, {round(state.imu.accelerometer[1], 6)}, {round(state.imu.accelerometer[2], 6)}")
                # rpy = Euler angle: 0 = Roll; 1 = Pitch; 2 = Yaw
                print(f"rpy: {round(state.imu.rpy[0], 6)}, {round(state.imu.rpy[1], 6)}, {round(state.imu.rpy[2], 6)}")
                print(f"temperature = {state.imu.temperature}")
                print("\n")

            mode_ar.append(state.mode)
            bodyHeight_ar.append(state.bodyHeight)
            footRaiseHeight_ar.append(state.footRaiseHeight)
            yawSpeed_ar.append(state.yawSpeed)
            footForce_ar.append([state.footForce[0], state.footForce[1], state.footForce[2], state.footForce[3]])
            velocity_ar.append([state.velocity[0], state.velocity[1], state.velocity[2]])

            gyroscope_ar.append([state.imu.gyroscope[0],state.imu.gyroscope[1],state.imu.gyroscope[2]])
            accelerometer_ar.append([state.imu.accelerometer[0],state.imu.accelerometer[1],state.imu.accelerometer[2]])
            rpy_ar.append([state.imu.rpy[0],state.imu.rpy[1],state.imu.rpy[2]])
            temperature_ar.append(state.imu.temperature)

            
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

        if motiontime > 15 / sleep_in_seconds:
            # convert the arrays and save them
            mode_ar = np.asarray(mode_ar)
            np.savetxt("logged_data/mode.csv", mode_ar, delimiter=";")

            bodyHeight_ar = np.asarray(bodyHeight_ar)
            np.savetxt("logged_data/bodyHeight.csv", bodyHeight_ar, delimiter=";")

            footRaiseHeight_ar = np.asarray(footRaiseHeight_ar)
            np.savetxt("logged_data/footRaiseHeight.csv", footRaiseHeight_ar, delimiter=";")

            yawSpeed_ar = np.asarray(yawSpeed_ar)
            np.savetxt("logged_data/yawSpeed.csv", yawSpeed_ar, delimiter=";")

            footForce_ar = np.asarray(footForce_ar)
            np.savetxt("logged_data/footForce.csv", footForce_ar, delimiter=";")

            velocity_ar = np.asarray(velocity_ar)
            np.savetxt("logged_data/velocity.csv", velocity_ar, delimiter=";")

            gyroscope_ar = np.asarray(gyroscope_ar)
            np.savetxt("logged_data/gyroscope.csv", gyroscope_ar, delimiter=";")

            accelerometer_ar = np.asarray(accelerometer_ar)
            np.savetxt("logged_data/accelerometer.csv", accelerometer_ar, delimiter=";")

            rpy_ar = np.asarray(rpy_ar)
            np.savetxt("logged_data/rpy.csv", rpy_ar, delimiter=";")

            temperature_ar = np.asarray(temperature_ar)
            np.savetxt("logged_data/temperature.csv", temperature_ar, delimiter=";")
            
            sys.exit(0)
     
        udp.SetSend(cmd)
        udp.Send()
