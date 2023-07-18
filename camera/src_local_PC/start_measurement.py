import paramiko
import threading
import time

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


if __name__ == "__main__":
    Nano13_thread = threading.Thread(target=start_camera_measurement_via_ssh, args=(13, ))
    Nano14_thread = threading.Thread(target=start_camera_measurement_via_ssh, args=(14, ))
    Nano15_thread = threading.Thread(target=start_camera_measurement_via_ssh, args=(15, ))

    Nano13_thread.start()
    Nano14_thread.start()
    Nano15_thread.start()

    Nano13_thread.join()
    Nano14_thread.join()
    Nano15_thread.join()
