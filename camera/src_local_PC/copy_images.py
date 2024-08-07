import subprocess
import os


def copy_captured_images(ip_last_segment, destination):
    """
        Function to copy measurement data from a Unitree Go1 Nano to destination.
        NOTE: Your system must be able to ping the Nano directly (connected via LAN)!

        Parameters:
            - ip_last_segment (int): Last number of the IPv4 address for the SSH connection
            - destination (str): Path to the dir where data shall be copied to
    """
    target = f"unitree@192.168.123.{ip_last_segment}:/home/unitree/Unitree/sdk/UnitreeCameraSdk/data"

    print(f"Start copy measurement files from Nano {ip_last_segment}")
    p = subprocess.Popen(["scp", "-r", target, destination])
    sts = os.waitpid(p.pid, 0)
    print("Copying finished")


def copy_captured_images_with_sshpass(ip_last_segment, destination, delete_files_on_remote=False):
    """
        Function to copy measurement data from a Unitree Go1 Nano to destination.
        NOTE: Your system must be able to ping the Nano directly (connected via LAN)!

        Parameters:
            - ip_last_segment (int): Last number of the IPv4 address for the SSH connection
            - destination (str): Path to the dir where data shall be copied to
            - delete_files_on_remote (bool): Default = False. If set to True, files on the SSH host will be deleted.
    """
    ssh_target = f"unitree@192.168.123.{ip_last_segment}"
    target_dir = "/home/unitree/Unitree/sdk/UnitreeCameraSdk/data/"
    target = ssh_target + ":" + target_dir

    print(f"Start copy measurement files from Nano {ip_last_segment}")
    p = subprocess.Popen(
        ["sshpass", "-p", "123", "scp", "-r", target, destination])
    sts = os.waitpid(p.pid, 0)

    if delete_files_on_remote:
        p = subprocess.Popen(
            ["sshpass", "-p", "123", "ssh", ssh_target, "rm", "-r", target_dir])
        sts = os.waitpid(p.pid, 0)
        print("Copying (including removing files on remote) finished")
    else:
        print("Copying finished")


if __name__ == "__main__":
    destination = "/home/eissen/measurements"

    copy_captured_images_with_sshpass(13, destination, True)
    copy_captured_images_with_sshpass(14, destination, True)
    copy_captured_images_with_sshpass(15, destination, True)
