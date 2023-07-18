import subprocess
import os

def copy_captured_images(ip_last_segment, destination):
    """
        Function to copy measurement data from a Unitree Go1 Nano to destination.
        NOTE: Your system must be able to ping the Nano directly (connected via LAN)!
    """
    target = f"unitree@192.168.123.{ip_last_segment}:/home/unitree/Unitree/sdk/UnitreeCameraSdk/data"
    
    print(f"Start copy measurement files from Nano {ip_last_segment}")
    p = subprocess.Popen(["scp", "-r", target, destination])
    sts = os.waitpid(p.pid, 0)
    print("Copying finished successfully!")

def copy_captured_images_with_sshpass(ip_last_segment, destination):
    """
        Function to copy measurement data from a Unitree Go1 Nano to destination.
        NOTE: Your system must be able to ping the Nano directly (connected via LAN)!
    """
    target = f"unitree@192.168.123.{ip_last_segment}:/home/unitree/Unitree/sdk/UnitreeCameraSdk/data/"
    
    print(f"Start copy measurement files from Nano {ip_last_segment}")
    p = subprocess.Popen(["sshpass", "-p", "123", "scp", "-r", target, destination])
    sts = os.waitpid(p.pid, 0)
    print("Copying finished successfully!")

if __name__ == "__main__":
    destination = "/home/eissen/measurements"

    # copy_captured_images(13, destination)
    # copy_captured_images(14, destination)
    # copy_captured_images(15, destination)

    copy_captured_images_with_sshpass(13, destination)
    copy_captured_images_with_sshpass(14, destination)
    copy_captured_images_with_sshpass(15, destination)
