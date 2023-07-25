import read_IMU_data.scripts.read_IMU as read_IMU
import camera.src_local_PC.start_measurement as camera_measurement
import camera.src_local_PC.copy_images as copy_images
from datetime import datetime, timedelta
import threading
from custom_utils.utils import save_struct_as_json, get_floor_type_from_user


if __name__ == "__main__":
    # variables for main
    running = threading.Event()
    running.set()
    iterations_time_diff_calculation = 10
    info_struct = {} # struct to add all data which shall be stored in injo.json
    measurement_base_path_for_copy = "/home/eissen/measurements"
    measurement_base_path = measurement_base_path_for_copy + "/data"

    # store date of measurement
    measurement_date = datetime.now().strftime("%d.%m.%Y")
    info_struct["measurement_date"] = measurement_date

    # get user input about floor type to be logged in info.json
    floor_type = get_floor_type_from_user()
    info_struct["floor type"] = floor_type

    # get time diff for all Go1 µCs and save them in the info_struct
    time_diff_13, corrected_time_diff_13, duration_mean_13, later_timestamp_13 = camera_measurement.get_average_time_diff_ms(13, "unitree", iterations_time_diff_calculation, True)
    time_diff_14, corrected_time_diff_14, duration_mean_14, later_timestamp_14 = camera_measurement.get_average_time_diff_ms(14, "unitree", iterations_time_diff_calculation, True)
    time_diff_15, corrected_time_diff_15, duration_mean_15, later_timestamp_15 = camera_measurement.get_average_time_diff_ms(15, "unitree", iterations_time_diff_calculation, True)
    time_diff_pi, corrected_time_diff_pi, duration_mean_pi, later_timestamp_pi = camera_measurement.get_average_time_diff_ms(161, "pi", iterations_time_diff_calculation, True)

    info_struct["time_diff_13_in_ms"] = {"normal": time_diff_13, "corrected": corrected_time_diff_13, "duration": duration_mean_13, "later timestamp on": later_timestamp_13}
    info_struct["time_diff_14_in_ms"] = {"normal": time_diff_14, "corrected": corrected_time_diff_14, "duration": duration_mean_14, "later timestamp on": later_timestamp_14}
    info_struct["time_diff_15_in_ms"] = {"normal": time_diff_15, "corrected": corrected_time_diff_15, "duration": duration_mean_15, "later timestamp on": later_timestamp_15}
    info_struct["time_diff_pi_in_ms"] = {"normal": time_diff_pi, "corrected": corrected_time_diff_pi, "duration": duration_mean_pi, "later timestamp on": later_timestamp_pi}

    # set starting time for all threads to 30 seconds in the future
    start_time = datetime.now() + timedelta(seconds=15)
    start_time_string = start_time.strftime("%H:%M:%S")
    info_struct["starting_time"] = start_time_string
    
    # create and start all measurement threads to get all data in parallel
    imu_thread = read_IMU.ReadImuDataGo1(running, use_LAN=True, measurement_base_path=measurement_base_path, starting_time=start_time)
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

    # log end time of the measurement
    end_time = datetime.now()
    end_time_string = end_time.strftime("%H:%M:%S")
    info_struct["end_time"] = end_time_string

    # save info_struct to info.json in measurement dir
    save_struct_as_json(imu_thread.path_measurement_dir, "info.json", info_struct)

    print("Measurement was stopped properly, images will be copied now.")

    copy_images.copy_captured_images_with_sshpass(13, measurement_base_path_for_copy, True)
    copy_images.copy_captured_images_with_sshpass(14, measurement_base_path_for_copy, True)
    copy_images.copy_captured_images_with_sshpass(15, measurement_base_path_for_copy, True)