import paramiko
import threading
from datetime import datetime, timedelta
import os
import numpy as np
import subprocess

def execute_command_via_ssh(ip_addr, pwd, command):
    """
        Function to start a SSH session to "unitree@<ip_addr> and uses the password <pwd>. Afterwards <command> will be executed on the connected machine and it's output printed.
    """
    # create SSH connection
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(ip_addr, username='unitree', password=pwd)

    # Execute command. The triggered process will be automatically ended in case the SSH connection is closed
    stdin, stdout, stderr = client.exec_command(command, get_pty=True)

    # print output of process
    for line in stdout:
        print(f"Update for {ip_addr}: {line}")

    client.close()

def start_camera_measurement_via_ssh(ip_last_segment, starting_time_string):
    """
        Function to start a SSH session to "unitree@192.168.123.{ip_last_segment}. Afterwards the camera measurement is started and it's output printed.
        NOTE: The triggered processes will be automatically ended in case the SSH connection is closed
    """
    # create SSH connection
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(f"192.168.123.{ip_last_segment}", username='unitree', password="123")

    
    commands = [] # list for all commands, as they must be sent at once to keep the same shell for it
    commands.append("/home/unitree/Documents/killCameraProcesses.sh") # command to kill camera processes by executing killCameraProcesses.sh on the Nanos
    commands.append("cd /home/unitree/Unitree/sdk/UnitreeCameraSdk") # command to change directory to UnitreeCameraSdk
    
    # append command to execute camera executable
    if ip_last_segment == 15:
        # Nano with IP 15 has only one camera
        commands.append("./bins/getFrameOneCamera")
    else:
        commands.append("./bins/getFrameTwoCameras")

    commands.append(starting_time_string)

    # prepare command string and execute the commands
    command_string = "; ".join(commands)
    stdin, stdout, stderr = client.exec_command(command_string, get_pty=True) 

    # print output of process
    for line in stdout:
        print(f"Update for {ip_last_segment}: {line}")

    client.close()


def set_time_via_ssh_for_Nano(target_ip_last_segment, user_time_source):
    """
        NOT WORKING YET!! due to missing possiblity to provide passwords!
        TODO: Update docstring
        NOTE: Your system must be able to ping the Nano directly (connected via LAN)!
    """
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(f"192.168.123.{target_ip_last_segment}", username='unitree', password="123")

    # prepare command string and execute the commands
    print(f"Set clock for Nano {target_ip_last_segment}")
    stdin, stdout, stderr = client.exec_command("/home/unitree/Documents/set_time.sh", get_pty=True) 

    # print output of process
    for line in stdout:
        print(f"Update for {target_ip_last_segment}: {line}")

    client.close()

def set_time_via_ssh_for_PI(user_time_source, ip_time_source):
    """
        NOT WORKING YET!! due to missing possiblity to provide passwords!
        TODO: Update docstring
        NOTE: Your system must be able to ping the Nano directly (connected via LAN)!
    """
    ssh_target = f"pi@192.168.123.161"
    date_format = f'--set="$(ssh {user_time_source}@{ip_time_source} date \\"+%C%y-%m-%d %H:%M:%S\\")"'

    print(f"Set clock for Pi")
    p = subprocess.Popen(["sshpass", "-p", "123", "ssh", ssh_target, "sudo", "date", date_format])
    sts = os.waitpid(p.pid, 0)

def get_time_diff(ip_last_segment, remote_username, print_info=False):
    # ideas:
    #  - correct time diff by duration, e.g. add 1/3 of duration to start_time
    #  - determine time_diff for multiple times (e.g. 10 times), as result is not always the same
    date_format_string ='date +"%C%y-%m-%d %T.%6N"' # format string for date shell command

    # create SSH session to get time from remote PC
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(f"192.168.123.{ip_last_segment}", username=remote_username, password='123')

    start_time = datetime.now() # get time on local PC
    stdin, stdout, stderr = client.exec_command(date_format_string, get_pty=True) # get time on remote PC
    end_time = datetime.now() # get time on local PC again to determine duration

    # convert time of target from stdout and remove \r\n at end of line
    for line in stdout:
        time_of_remote = line.replace("\n", "").replace("\r", "")

    client.close()

    # convert time_of_remote to datetime object
    time_of_remote = datetime.strptime(time_of_remote, '%Y-%m-%d %H:%M:%S.%f')

    # calculate duration and corrected start time which will be increased by 1/3 of the duration
    duration = end_time - start_time
    corrected_start_time = start_time + (duration / 3)

    # calculate time diff where always the earlier time must be subtracted from the later time
    if (start_time > time_of_remote) and (corrected_start_time > time_of_remote):
        later_timestamp = "local PC"
        time_diff = (start_time - time_of_remote)
        corrected_time_diff = (corrected_start_time - time_of_remote)
    elif (start_time > time_of_remote) and (corrected_start_time < time_of_remote):
        later_timestamp = "local PC (only uncorrected)"
        time_diff = (start_time - time_of_remote)
        corrected_time_diff = (time_of_remote - corrected_start_time)
    elif (start_time < time_of_remote) and (corrected_start_time < time_of_remote):
        later_timestamp = "remote PC"
        time_diff = (time_of_remote - start_time)
        corrected_time_diff = (time_of_remote - corrected_start_time)
    else:
        # should not be possible, thus throw and exception
        raise Exception("Problem with timediff calculation.")

    if print_info:
        print(f"{later_timestamp} is later timestamp by: {time_diff}, corrected timediff is {corrected_time_diff}\nOperation took {duration}")

    return time_diff, corrected_time_diff, duration, start_time, time_of_remote, later_timestamp

def get_average_time_diff_ms(ip_last_segment, remote_username, iterations, print_info=False):
    time_diff_ar = []
    corrected_time_diff_ar = []
    duration_ar = []
    start_time_ar = []
    time_of_remote_ar = []
    evaluated_later_timestamp = ""

    for _ in range(iterations):
        time_diff, corrected_time_diff, duration, start_time, time_of_remote, later_timestamp = get_time_diff(
            ip_last_segment, remote_username)
        time_diff_ar.append(convert_timedelta_to_ms(time_diff))
        corrected_time_diff_ar.append(convert_timedelta_to_ms(corrected_time_diff))
        duration_ar.append(convert_timedelta_to_ms(duration))
        start_time_ar.append(start_time)
        time_of_remote_ar.append(time_of_remote)
        # check if entry for later timestamp is the same always
        if later_timestamp != evaluated_later_timestamp and evaluated_later_timestamp != "":
            evaluated_later_timestamp = "inconsistent"
        else:
            evaluated_later_timestamp = later_timestamp

    # calculate average and standard deviation for each value using numpy
    time_diff_ar = np.asarray(time_diff_ar)
    corrected_time_diff_ar = np.asarray(corrected_time_diff_ar)
    duration_ar = np.asarray(duration_ar)

    time_diff_mean = np.mean(time_diff_ar)
    time_diff_std = np.std(time_diff_ar)
    corrected_time_diff_mean = np.mean(corrected_time_diff_ar)
    corrected_time_diff_std = np.std(corrected_time_diff_ar)
    duration_mean = np.mean(duration_ar)
    duration_std = np.std(duration_ar)

    if print_info:
        print(
            f"For {ip_last_segment} the mean time diff is: {time_diff_mean:.3f} +- {time_diff_std:.3f} ms")
        print(f"For {ip_last_segment} the mean corrected time diff is: {corrected_time_diff_mean:.3f} +- {corrected_time_diff_std:.3f} ms")
        print(
            f"For {ip_last_segment} the mean duration is: {duration_mean:.3f} +- {duration_std:.3f} ms")

    return time_diff_mean, corrected_time_diff_mean, evaluated_later_timestamp

def convert_timedelta_to_ms(timedelta_to_convert: timedelta):
    # timedelta stores time split in microsecond, seconds an days -> for total ms all three must be converted to ms
    return timedelta_to_convert.microseconds/1000 + timedelta_to_convert.seconds * 1000 + timedelta_to_convert.days * 24 * 60 * 60 * 1000

if __name__ == "__main__":
    # variables for main
    running = True
    set_time = False
    user_time_source = "eissen"
    ip_time_source = "192.168.123.52"
    iterations_time_diff_calculation = 10

    # first set time for all µCs if selected by set_time flag
    # NOTE: Not possible yet, instead run the file /home/unitree/Documents/set_time.sh on the remote by your own!
    # if set_time:
    #     set_time_via_ssh_for_Nano(13, user_time_source)
    #     set_time_via_ssh_for_Nano(14, user_time_source, ip_time_source)
    #     set_time_via_ssh_for_Nano(15, user_time_source, ip_time_source)
    #     set_time_via_ssh_for_PI(user_time_source, ip_time_source)

    # TODO: do someting with time_diff, most likely log it and correct timestamps afterwards
    time_diff_13, corrected_time_diff_13, later_timestamp_13 = get_average_time_diff_ms(
        13, "unitree", iterations_time_diff_calculation, True)
    time_diff_14, corrected_time_diff_14, later_timestamp_14 = get_average_time_diff_ms(
        14, "unitree", iterations_time_diff_calculation, True)
    time_diff_15, corrected_time_diff_15, later_timestamp_15 = get_average_time_diff_ms(
        15, "unitree", iterations_time_diff_calculation, True)
    time_diff_pi, corrected_time_diff_pi, later_timestamp_pi = get_average_time_diff_ms(
        161, "pi", iterations_time_diff_calculation, True)

    # set starting time for all threads to 30 seconds in the future
    start_time = datetime.now() + timedelta(seconds=30)
    start_time_string = start_time.strftime("%H:%M:%S")

    # create and start all measurement threads to get all data in parallel
    Nano13_thread = threading.Thread(
        target=start_camera_measurement_via_ssh, args=(13, start_time_string, ))
    Nano14_thread = threading.Thread(
        target=start_camera_measurement_via_ssh, args=(14, start_time_string, ))
    Nano15_thread = threading.Thread(
        target=start_camera_measurement_via_ssh, args=(15, start_time_string, ))

    # set Nano threads to deamon threads, so they will be automatically killed when imu_thread ends
    Nano13_thread.daemon = True
    Nano14_thread.daemon = True
    Nano15_thread.daemon = True

    Nano13_thread.start()
    Nano14_thread.start()
    Nano15_thread.start()

    # check whether measurement shall be stopped
    while running:
        user_input = input("\n!!!! Enter stop to end the measurement !!!!\n")
        if "stop" in user_input.lower():
            running = False

    print("Measurement was stopped properly!")
