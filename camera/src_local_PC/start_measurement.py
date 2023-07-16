import paramiko
import threading

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

if __name__ == "__main__":
    Nano13_thread = threading.Thread(target=execute_command_via_ssh, args=("192.168.123.13", '123', 'TODO'))
    Nano14_thread = threading.Thread(target=execute_command_via_ssh, args=("192.168.123.14", '123', 'TODO'))
    Nano15_thread = threading.Thread(target=execute_command_via_ssh, args=("192.168.123.15", '123', 'TODO'))

    Nano13_thread.start()
    Nano14_thread.start()
    Nano15_thread.start()

    Nano13_thread.join()
    Nano14_thread.join()
    Nano15_thread.join()
