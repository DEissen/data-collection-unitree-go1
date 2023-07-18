import read_IMU_data.scripts.read_IMU as read_IMU
import camera.src_local_PC.start_measurement as camera_measurement

import threading


if __name__ == "__main__":
    IMU_thread = threading.Thread(target=read_IMU.read_IMU_data, args=())
    Nano13_thread = threading.Thread(target=camera_measurement.start_camera_measurement_via_ssh, args=(13, ))
    Nano14_thread = threading.Thread(target=camera_measurement.start_camera_measurement_via_ssh, args=(14, ))
    Nano15_thread = threading.Thread(target=camera_measurement.start_camera_measurement_via_ssh, args=(15, ))

    IMU_thread.start()
    Nano13_thread.start()
    Nano14_thread.start()
    Nano15_thread.start()

    IMU_thread.join()
    Nano13_thread.join()
    Nano14_thread.join()
    Nano15_thread.join()