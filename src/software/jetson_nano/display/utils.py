import subprocess


def get_ip_address():
    """ Returns the IP address of the roboot """
    try:
        cmd = "hostname -I | cut -d' ' -f1"
        IP = subprocess.check_output(cmd, shell=True).decode("utf-8")
    except:
        IP = "N/A"
    return IP


def get_signal_strength():
    """ Returns the signal strength of the robot """
    try:
        cmd = "iwconfig | grep 'Signal level='"
        signal_strength = (
            subprocess.check_output(cmd, stderr=subprocess.STDOUT, shell=True)
            .decode("utf-8")
            .split("Signal level=")[1]
            .replace("\n", "")
        )
    except:
        signal_strength = "N/A"
    return signal_strength
