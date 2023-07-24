#!/usr/bin/python

import sys
import time
import os
import numpy as np
import threading
from datetime import datetime, timedelta

# get path to robot_interface lib from Unitree
file_dir = os.path.dirname(os.path.abspath(__file__))
path_to_append = os.path.join(file_dir, os.pardir, "lib/python/amd64") # last part might needed to be changed to "../lib/python/arm64" on other system

sys.path.append(path_to_append)
import robot_interface as sdk  # nopep8


class ReadImuDataGo1(threading.Thread):
    def __init__(self, running, use_LAN, starting_time: datetime):
        super().__init__() # call init of super class to enable usage as thread
        # initialize members
        self.runCounter = 0
        self.info_printed_once = False
        self.start_logging = False
        self.show_print = False
        # all data seems to be sampled with 50 Hz (9 - 10 equal measurements when self.sleep_in_seconds is 0.002)
        # => use at least 0.01 as self.sleep_in_seconds
        self.sleep_in_seconds = 0.01
        self.measurement_duration = 600
        self.log_interval = 10
        self.measurement_timestamp = 0

        self.running = running
        self.starting_time = starting_time

        self.HIGHLEVEL = 0xee
        self.LOWLEVEL = 0xff

        # lists to log the data
        self.mode_ar = []
        self.bodyHeight_ar = []
        self.footRaiseHeight_ar = []
        self.yawSpeed_ar = []
        self.footForce_ar = []
        self.velocity_ar = []
        self.gyroscope_ar = []
        self.accelerometer_ar = []
        self.rpy_ar = []
        self.temperature_ar = []

        # create measurement directory and get paths for the sensors
        self.create_measurement_folder()

        if use_LAN:
            self.ip_addr ="192.168.123.161"
        else:
            self.ip_addr ="192.168.12.1"

        print(f"Start measurement with Pi {self.ip_addr}")
        self.udp = sdk.UDP(self.HIGHLEVEL, 8080, self.ip_addr, 8082)

        self.cmd = sdk.HighCmd()
        self.state = sdk.HighState()
        self.udp.InitCmdData(self.cmd)

    def run(self):
        while self.running.is_set():
            # check if logging shall already start
            if datetime.now() > self.starting_time:
                time.sleep(self.sleep_in_seconds)

                self.udp.Recv()
                self.udp.GetRecv(self.state)

                # print general info once at the beginning after a short waiting time
                if not self.info_printed_once and self.runCounter > 1 / self.sleep_in_seconds:
                    print(f"Header = {self.state.head}")
                    print(f"levelFlag = {self.state.levelFlag}")
                    print(f"frameReserve = {self.state.frameReserve}")
                    print(f"SN = {self.state.SN}")
                    print(f"version = {self.state.version}")
                    print(f"bandwidth = {self.state.bandWidth}")
                    print(f"crc = {self.state.crc}")
                    self.info_printed_once = True
                    self.measurement_timestamp = datetime.now().strftime("%H_%M_%S_%f")[:-3]
                    self.start_logging = True

                # log everything that seems to be interesting every iteration after general info was logged once
                if self.start_logging:
                    if self.show_print:
                        print(f"logging for step {self.runCounter}")
                        print(f"mode = {self.state.mode}")
                        print(f"bodyHeight = {round(self.state.bodyHeight, 6)}")
                        print(f"footRaiseHeight = {round(self.state.footRaiseHeight, 6)}")
                        # yawSpeed = rotation speed of robot
                        print(f"yawSpeed = {round(self.state.yawSpeed, 6)}")
                        # meaning of footForce: 0 = vorne rechts; 1 = vorne links; 2 = hinten rechts; 3 = hinten links
                        print(
                            f"footForce: {round(self.state.footForce[0], 6)}, {round(self.state.footForce[1], 6)}, {round(self.state.footForce[2], 6)}, {round(self.state.footForce[3], 6)}")
                        # footForceEst is useless
                        print(
                            f"footForceEst: {round(self.state.footForceEst[0], 6)}, {round(self.state.footForceEst[1], 6)}, {round(self.state.footForceEst[2], 6)}, {round(self.state.footForceEst[3], 6)}")
                        # position does not seem to be useful
                        print(
                            f"position: {round(self.state.position[0], 6)}, {round(self.state.position[1], 6)}, {round(self.state.position[2], 6)}")
                        # meaning of velocity: 0 = vorwärts/ rückwärts; 1 = seitwärts; 2 = hoch/runter
                        print(
                            f"velocity: {round(self.state.velocity[0], 6)}, {round(self.state.velocity[1], 6)}, {round(self.state.velocity[2], 6)}")
                        print("IMU data:")
                        # gyroscope: 0 = x; 1 = y; 2 = z
                        print(
                            f"gyroscope: {round(self.state.imu.gyroscope[0], 6)}, {round(self.state.imu.gyroscope[1], 6)}, {round(self.state.imu.gyroscope[2], 6)}")
                        # accelerometer: 0 = x; 1 = y; 2 = z
                        print(
                            f"accelerometer: {round(self.state.imu.accelerometer[0], 6)}, {round(self.state.imu.accelerometer[1], 6)}, {round(self.state.imu.accelerometer[2], 6)}")
                        # rpy = Euler angle: 0 = Roll; 1 = Pitch; 2 = Yaw
                        print(
                            f"rpy: {round(self.state.imu.rpy[0], 6)}, {round(self.state.imu.rpy[1], 6)}, {round(self.state.imu.rpy[2], 6)}")
                        print(f"temperature = {self.state.imu.temperature}")
                        print("\n")

                    self.mode_ar.append(self.state.mode)
                    self.bodyHeight_ar.append(self.state.bodyHeight)
                    self.footRaiseHeight_ar.append(self.state.footRaiseHeight)
                    self.yawSpeed_ar.append(self.state.yawSpeed)
                    self.footForce_ar.append(
                        [self.state.footForce[0], self.state.footForce[1], self.state.footForce[2], self.state.footForce[3]])
                    self.velocity_ar.append(
                        [self.state.velocity[0], self.state.velocity[1], self.state.velocity[2]])
                    self.gyroscope_ar.append(
                        [self.state.imu.gyroscope[0], self.state.imu.gyroscope[1], self.state.imu.gyroscope[2]])
                    self.accelerometer_ar.append(
                        [self.state.imu.accelerometer[0], self.state.imu.accelerometer[1], self.state.imu.accelerometer[2]])
                    self.rpy_ar.append(
                        [self.state.imu.rpy[0], self.state.imu.rpy[1], self.state.imu.rpy[2]])
                    self.temperature_ar.append(self.state.imu.temperature)

                # log in the specified interval
                if self.start_logging and ((self.runCounter * self.sleep_in_seconds) % self.log_interval == 0):
                    self.save_logs()

                # end measurement after specified time
                if self.runCounter > self.measurement_duration / self.sleep_in_seconds:
                    sys.exit(0)

                # prepare and send high level command to do nothing!
                self.cmd.mode = 0      # 0:idle, default stand      1:forced stand     2:walk continuously
                self.cmd.gaitType = 0
                self.cmd.speedLevel = 0
                self.cmd.footRaiseHeight = 0
                self.cmd.bodyHeight = 0
                self.cmd.euler = [0, 0, 0]
                self.cmd.velocity = [0, 0]
                self.cmd.yawSpeed = 0.0
                self.cmd.reserve = 0

                self.udp.SetSend(self.cmd)
                self.udp.Send()

                self.runCounter = self.runCounter + 1

            # wait some time for start of logging
            else:
                time.sleep(self.sleep_in_seconds)

        # save logs in case running is not set anymore by external source to not miss any data
        self.save_logs()


    def save_logs(self):
        # convert the lists to numpy arrays and save them
        self.mode_ar = np.asarray(self.mode_ar)
        np.savetxt(os.path.join(self.mode_data_dir,
                f"{self.measurement_timestamp}.csv"), self.mode_ar, delimiter=";")

        self.bodyHeight_ar = np.asarray(self.bodyHeight_ar)
        np.savetxt(os.path.join(self.bodyHeight_data_dir, f"{self.measurement_timestamp}.csv"),
                self.bodyHeight_ar, delimiter=";")

        self.footRaiseHeight_ar = np.asarray(self.footRaiseHeight_ar)
        np.savetxt(os.path.join(self.footRaiseHeight_data_dir, f"{self.measurement_timestamp}.csv"),
                self.footRaiseHeight_ar, delimiter=";")

        self.yawSpeed_ar = np.asarray(self.yawSpeed_ar)
        np.savetxt(os.path.join(
            self.yawSpeed_data_dir, f"{self.measurement_timestamp}.csv"), self.yawSpeed_ar, delimiter=";")

        self.footForce_ar = np.asarray(self.footForce_ar)
        np.savetxt(os.path.join(self.footForce_data_dir, f"{self.measurement_timestamp}.csv"),
                self.footForce_ar, delimiter=";")

        self.velocity_ar = np.asarray(self.velocity_ar)
        np.savetxt(os.path.join(
            self.velocity_data_dir, f"{self.measurement_timestamp}.csv"), self.velocity_ar, delimiter=";")

        self.gyroscope_ar = np.asarray(self.gyroscope_ar)
        np.savetxt(os.path.join(self.gyroscope_data_dir, f"{self.measurement_timestamp}.csv"),
                self.gyroscope_ar, delimiter=";")

        self.accelerometer_ar = np.asarray(self.accelerometer_ar)
        np.savetxt(os.path.join(self.accelerometer_data_dir, f"{self.measurement_timestamp}.csv"),
                self.accelerometer_ar, delimiter=";")

        self.rpy_ar = np.asarray(self.rpy_ar)
        np.savetxt(os.path.join(
            self.rpy_data_dir, f"{self.measurement_timestamp}.csv"), self.rpy_ar, delimiter=";")

        self.temperature_ar = np.asarray(self.temperature_ar)
        np.savetxt(os.path.join(self.rpy_data_dir, f"{self.measurement_timestamp}.csv"),
                self.temperature_ar, delimiter=";")

        # reset lists for next measurement
        self.mode_ar = []
        self.bodyHeight_ar = []
        self.footRaiseHeight_ar = []
        self.yawSpeed_ar = []
        self.footForce_ar = []
        self.velocity_ar = []
        self.gyroscope_ar = []
        self.accelerometer_ar = []
        self.rpy_ar = []
        self.temperature_ar = []

        # get new measurement timestamp
        self.measurement_timestamp = datetime.now().strftime("%H_%M_%S_%f")[:-3]

    def create_measurement_folder(self):
        # create folder for results (if it does not exist yet)
        measurement_dir_timestamp = datetime.now().strftime("%d_%m__%H_%M")
        self.path_measurement_dir = f"./measurement_{measurement_dir_timestamp}"

        self.mode_data_dir = f"{self.path_measurement_dir}/mode"
        self.bodyHeight_data_dir = f"{self.path_measurement_dir}/bodyHeight"
        self.footRaiseHeight_data_dir = f"{self.path_measurement_dir}/footRaiseHeight"
        self.yawSpeed_data_dir = f"{self.path_measurement_dir}/yawSpeed"
        self.footForce_data_dir = f"{self.path_measurement_dir}/footForce"
        self.velocity_data_dir = f"{self.path_measurement_dir}/velocity"
        self.gyroscope_data_dir = f"{self.path_measurement_dir}/gyroscope"
        self.accelerometer_data_dir = f"{self.path_measurement_dir}/accelerometer"
        self.rpy_data_dir = f"{self.path_measurement_dir}/rpy"
        self.rpy_data_dir = f"{self.path_measurement_dir}/temperature"

        os.makedirs(self.mode_data_dir, exist_ok=True)
        os.makedirs(self.bodyHeight_data_dir, exist_ok=True)
        os.makedirs(self.footRaiseHeight_data_dir, exist_ok=True)
        os.makedirs(self.yawSpeed_data_dir, exist_ok=True)
        os.makedirs(self.footForce_data_dir, exist_ok=True)
        os.makedirs(self.velocity_data_dir, exist_ok=True)
        os.makedirs(self.gyroscope_data_dir, exist_ok=True)
        os.makedirs(self.accelerometer_data_dir, exist_ok=True)
        os.makedirs(self.rpy_data_dir, exist_ok=True)
        os.makedirs(self.rpy_data_dir, exist_ok=True)


if __name__ == '__main__':
    start_time = datetime.now() + timedelta(seconds=5)

    running = threading.Event()
    running.set()

    imu_thread = ReadImuDataGo1(running, use_LAN=True, starting_time=start_time)

    imu_thread.start()

    while running.is_set():
        user_input = input("\n!!!! Enter stop to end the measurement !!!!\n")
        if "stop" in user_input.lower():
            running.clear()
