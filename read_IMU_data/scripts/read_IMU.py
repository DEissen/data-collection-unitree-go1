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
    def __init__(self, running, use_LAN, measurement_base_path, starting_time: datetime):
        super().__init__()  # call init of super class to enable usage as thread
        # initialize members
        self.runCounter = 0
        self.info_printed_once = False
        self.start_logging = False
        self.show_print = False
        # all data seems to be sampled with 50 Hz (9 - 10 equal measurements when self.sleep_in_seconds is 0.002)
        # => use at least 0.01 as self.sleep_in_seconds
        self.sleep_in_seconds = 0.01
        self.log_interval = 10
        self.measurement_timestamp = 0

        self.running = running
        # Thread must start one second earlier, as the programm will wait one second till it starts logging
        self.starting_time = starting_time - timedelta(seconds=1)

        self.HIGHLEVEL = 0xee
        self.LOWLEVEL = 0xff

        # lists to log the data
        self.mode_ar = []
        self.bodyHeight_ar = []
        self.yawSpeed_ar = []
        self.footForce_ar = []
        self.velocity_ar = []
        self.gyroscope_ar = []
        self.accelerometer_ar = []
        self.rpy_ar = []

        # list of lists to store the data at the end
        self.mode_storage_dict = {}
        self.bodyHeight_storage_dict = {}
        self.yawSpeed_storage_dict = {}
        self.footForce_storage_dict = {}
        self.velocity_storage_dict = {}
        self.gyroscope_storage_dict = {}
        self.accelerometer_storage_dict = {}
        self.rpy_storage_dict = {}

        # create measurement directory and get paths for the sensors
        self.create_measurement_folder(measurement_base_path)

        if use_LAN:
            self.ip_addr = "192.168.123.161"
        else:
            self.ip_addr = "192.168.12.1"

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
                    self.measurement_timestamp = datetime.now().strftime("%H_%M_%S_%f")[
                        :-3]
                    self.start_logging = True
                    print("\nIMU measurement starts now\n")

                # log everything that seems to be interesting every iteration after general info was logged once
                if self.start_logging:
                    if self.show_print:
                        print(f"logging for step {self.runCounter}")
                        print(f"mode = {self.state.mode}")
                        print(
                            f"bodyHeight = {round(self.state.bodyHeight, 6)}")
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
                        print("\n")

                    self.mode_ar.append(self.state.mode)
                    self.bodyHeight_ar.append(self.state.bodyHeight)
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

                # log in the specified interval
                if self.start_logging and ((self.runCounter * self.sleep_in_seconds) % self.log_interval == 0):
                    self.store_log_to_dats_dicts()

                # prepare and send high level command to do nothing!
                self.cmd.mode = 0      # 0:idle, default stand      1:forced stand     2:walk continuously
                self.cmd.gaitType = 0
                self.cmd.speedLevel = 0
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

        # store data persistently in case running is not set anymore by external source to not miss any data
        self.store_log_to_dats_dicts()
        self.store_data_persistently()

    def store_log_to_dats_dicts(self):
        # storing of logs is only possible if logging was started
        if self.start_logging:
            # convert the lists to numpy arrays and save them in storage dicts
            self.mode_ar = np.asarray(self.mode_ar)
            self.mode_storage_dict[self.measurement_timestamp] = self.mode_ar

            self.bodyHeight_ar = np.asarray(self.bodyHeight_ar)
            self.bodyHeight_storage_dict[self.measurement_timestamp] = self.bodyHeight_ar

            self.yawSpeed_ar = np.asarray(self.yawSpeed_ar)
            self.yawSpeed_storage_dict[self.measurement_timestamp] = self.yawSpeed_ar

            self.footForce_ar = np.asarray(self.footForce_ar)
            self.footForce_storage_dict[self.measurement_timestamp] = self.footForce_ar

            self.velocity_ar = np.asarray(self.velocity_ar)
            self.velocity_storage_dict[self.measurement_timestamp] = self.velocity_ar

            self.gyroscope_ar = np.asarray(self.gyroscope_ar)
            self.gyroscope_storage_dict[self.measurement_timestamp] = self.gyroscope_ar

            self.accelerometer_ar = np.asarray(self.accelerometer_ar)
            self.accelerometer_storage_dict[self.measurement_timestamp] = self.accelerometer_ar

            self.rpy_ar = np.asarray(self.rpy_ar)
            self.rpy_storage_dict[self.measurement_timestamp] = self.rpy_ar

            # reset lists for next measurement
            self.mode_ar = []
            self.bodyHeight_ar = []
            self.yawSpeed_ar = []
            self.footForce_ar = []
            self.velocity_ar = []
            self.gyroscope_ar = []
            self.accelerometer_ar = []
            self.rpy_ar = []

            # get new measurement timestamp
            self.measurement_timestamp = datetime.now().strftime("%H_%M_%S_%f")[
                :-3]

    def store_data_persistently(self):
        # store data to csv files for all keys in the storage dicts
        for _, timestamp in enumerate(self.mode_storage_dict.keys()):
            np.savetxt(os.path.join(self.mode_data_dir,
                                    f"{timestamp}.csv"), self.mode_storage_dict[timestamp], delimiter=";")

            np.savetxt(os.path.join(self.bodyHeight_data_dir,
                       f"{timestamp}.csv"), self.bodyHeight_storage_dict[timestamp], delimiter=";")

            np.savetxt(os.path.join(self.yawSpeed_data_dir,
                       f"{timestamp}.csv"), self.yawSpeed_storage_dict[timestamp], delimiter=";")

            np.savetxt(os.path.join(self.footForce_data_dir,
                       f"{timestamp}.csv"), self.footForce_storage_dict[timestamp], delimiter=";")

            np.savetxt(os.path.join(self.velocity_data_dir,
                       f"{timestamp}.csv"), self.velocity_storage_dict[timestamp], delimiter=";")

            np.savetxt(os.path.join(self.gyroscope_data_dir,
                       f"{timestamp}.csv"), self.gyroscope_storage_dict[timestamp], delimiter=";")

            np.savetxt(os.path.join(self.accelerometer_data_dir,
                       f"{timestamp}.csv"), self.accelerometer_storage_dict[timestamp], delimiter=";")

            np.savetxt(os.path.join(
                self.rpy_data_dir, f"{timestamp}.csv"), self.rpy_storage_dict[timestamp], delimiter=";")

    def create_measurement_folder(self, measurement_base_path):
        # create folder for results (if it does not exist yet)
        measurement_dir_timestamp = datetime.now().strftime("%d_%m__%H_%M")
        self.path_measurement_dir = f"{measurement_base_path}/measurement_{measurement_dir_timestamp}"

        self.mode_data_dir = f"{self.path_measurement_dir}/mode"
        self.bodyHeight_data_dir = f"{self.path_measurement_dir}/bodyHeight"
        self.yawSpeed_data_dir = f"{self.path_measurement_dir}/yawSpeed"
        self.footForce_data_dir = f"{self.path_measurement_dir}/footForce"
        self.velocity_data_dir = f"{self.path_measurement_dir}/velocity"
        self.gyroscope_data_dir = f"{self.path_measurement_dir}/gyroscope"
        self.accelerometer_data_dir = f"{self.path_measurement_dir}/accelerometer"
        self.rpy_data_dir = f"{self.path_measurement_dir}/rpy"

        os.makedirs(self.mode_data_dir, exist_ok=True)
        os.makedirs(self.bodyHeight_data_dir, exist_ok=True)
        os.makedirs(self.yawSpeed_data_dir, exist_ok=True)
        os.makedirs(self.footForce_data_dir, exist_ok=True)
        os.makedirs(self.velocity_data_dir, exist_ok=True)
        os.makedirs(self.gyroscope_data_dir, exist_ok=True)
        os.makedirs(self.accelerometer_data_dir, exist_ok=True)
        os.makedirs(self.rpy_data_dir, exist_ok=True)


if __name__ == '__main__':
    start_time = datetime.now() + timedelta(seconds=1)

    running = threading.Event()
    running.set()

    imu_thread = ReadImuDataGo1(
        running, use_LAN=True, measurement_base_path=".", starting_time=start_time)

    imu_thread.start()

    while running.is_set():
        user_input = input("\n!!!! Enter stop to end the measurement !!!!\n")
        if "stop" in user_input.lower():
            running.clear()
