import read_IMU_data.scripts.read_IMU as read_IMU
import camera.src_local_PC.start_measurement as camera_measurement

import threading


if __name__ == "__main__":
    running = threading.Event()
    running.set()

    imu_thread = read_IMU.ReadImuDataGo1(running, use_LAN=True)
    Nano13_thread = threading.Thread(target=camera_measurement.start_camera_measurement_via_ssh, args=(13, ))
    Nano14_thread = threading.Thread(target=camera_measurement.start_camera_measurement_via_ssh, args=(14, ))
    Nano15_thread = threading.Thread(target=camera_measurement.start_camera_measurement_via_ssh, args=(15, ))

    imu_thread.start()
    Nano13_thread.start()
    Nano14_thread.start()
    Nano15_thread.start()

    # check whether measurement shall be stopped
    while running.is_set():
        user_input = input("\n!!!! Enter stop to end the measurement !!!!\n")
        if "stop" in user_input.lower():
            running.clear()

    print("IMU Measurement thread was stopped properly! Please kill the other processes now")