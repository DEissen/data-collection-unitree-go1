import read_IMU_data.scripts.read_IMU as read_IMU
import camera.src_local_PC.start_measurement as camera_measurement
from datetime import datetime, timedelta
import threading


if __name__ == "__main__":
    start_time = datetime.now() + timedelta(seconds=30)
    start_time_string = start_time.strftime("%H:%M:%S")
    
    running = threading.Event()
    running.set()

    imu_thread = read_IMU.ReadImuDataGo1(running, use_LAN=True, starting_time=start_time)
    Nano13_thread = threading.Thread(target=camera_measurement.start_camera_measurement_via_ssh, args=(13, start_time_string, ))
    Nano14_thread = threading.Thread(target=camera_measurement.start_camera_measurement_via_ssh, args=(14, start_time_string, ))
    Nano15_thread = threading.Thread(target=camera_measurement.start_camera_measurement_via_ssh, args=(15, start_time_string, ))

    # set Nano threads to deamon threads, so they will be automatically killed when imu_thread ends
    Nano13_thread.daemon = True
    Nano14_thread.daemon = True
    Nano15_thread.daemon = True

    imu_thread.start()
    Nano13_thread.start()
    Nano14_thread.start()
    Nano15_thread.start()

    # check whether measurement shall be stopped
    while running.is_set():
        user_input = input("\n!!!! Enter stop to end the measurement !!!!\n")
        if "stop" in user_input.lower():
            running.clear()

    print("Measurement was stopped properly!")