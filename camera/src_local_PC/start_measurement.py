import paramiko
import threading
from datetime import datetime
import os
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

def start_camera_measurement_via_ssh(ip_last_segment):
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

    # prepare command string and execute the commands
    command_string = "; ".join(commands)
    stdin, stdout, stderr = client.exec_command(command_string, get_pty=True) 

    # print output of process
    for line in stdout:
        print(f"Update for {ip_last_segment}: {line}")

    client.close()


def set_time_via_ssh_for_Nano(target_ip_last_segment, user_time_source, ip_time_source):
    """
        TODO: Update docstring
        NOTE: Your system must be able to ping the Nano directly (connected via LAN)!
    """
    ssh_target = f"unitree@192.168.123.{target_ip_last_segment}"
    date_format = f'--set="$(ssh {user_time_source}@{ip_time_source} date \\"+%C%y-%m-%d %H:%M:%S\\")"'

    print(f"Set clock for Nano {target_ip_last_segment}")
    p = subprocess.Popen(["sshpass", "-p", "123", "ssh", ssh_target, "sudo", "date", date_format])
    sts = os.waitpid(p.pid, 0)

def set_time_via_ssh_for_PI(user_time_source, ip_time_source):
    """
        TODO: Update docstring
        NOTE: Your system must be able to ping the Nano directly (connected via LAN)!
    """
    ssh_target = f"pi@192.168.123.161"
    date_format = f'--set="$(ssh {user_time_source}@{ip_time_source} date \\"+%C%y-%m-%d %H:%M:%S\\")"'

    print(f"Set clock for Pi")
    p = subprocess.Popen(["sshpass", "-p", "123", "ssh", ssh_target, "sudo", "date", date_format])
    sts = os.waitpid(p.pid, 0)

def get_time_diff(ip_last_segment, remote_username):
    # ideas:
    #  - correct time diff by duration, e.g. add 1/3 of duration to start_time
    #  - determine timedelta for multiple times (e.g. 10 times), as result is not always the same
    date_format_string ='date +"%C%y-%m-%d %T.%6N"' # format string for date shell command

    # create SSH session to get time from remote PC
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(f"192.168.185.{ip_last_segment}", username=remote_username, password='123')

    start_time = datetime.now() # get time on local PC
    stdin, stdout, stderr = client.exec_command(date_format_string, get_pty=True) # get time on remote PC
    end_time = datetime.now() # get time on local PC again to determine duration

    # convert time of target from stdout and remove \r\n at end of line
    for line in stdout:
        time_of_remote = line.replace("\n", "").replace("\r", "")

    client.close()

    # convert time_of_remote to datetime object
    time_of_remote = datetime.strptime(time_of_remote, '%Y-%m-%d %H:%M:%S.%f')
    
    duration = end_time - start_time
    
    if start_time > time_of_remote:
        later_timestamp = "local PC"
        timedelta = (start_time - time_of_remote)
    else:
        later_timestamp = "remote PC"
        timedelta = time_of_remote - start_time

    print(start_time)
    print(time_of_remote)
    print(f"{later_timestamp} is later timestamp by: {timedelta}\nOperation took {duration}")

    return timedelta


if __name__ == "__main__":

    # variables for setting time
    running = True
    set_time = False
    user_time_source = "eissen"
    ip_time_source = "192.168.123.52"

    # first set time for all µCs if selected by set_time flag
    if set_time:
        set_time_via_ssh_for_Nano(13, user_time_source, ip_time_source)
        set_time_via_ssh_for_Nano(14, user_time_source, ip_time_source)
        set_time_via_ssh_for_Nano(15, user_time_source, ip_time_source)
        set_time_via_ssh_for_PI(user_time_source, ip_time_source)

    # TODO: do someting with timedelta
    timedelta_13 = get_time_diff(13, "unitree")
    timedelta_14 = get_time_diff(14, "unitree")
    timedelta_15 = get_time_diff(15, "unitree")
    timedelta_pi = get_time_diff(201, "pi")

    Nano13_thread = threading.Thread(target=start_camera_measurement_via_ssh, args=(13, ))
    Nano14_thread = threading.Thread(target=start_camera_measurement_via_ssh, args=(14, ))
    Nano15_thread = threading.Thread(target=start_camera_measurement_via_ssh, args=(15, ))

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
