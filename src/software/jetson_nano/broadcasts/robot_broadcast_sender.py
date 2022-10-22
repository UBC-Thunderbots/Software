import argparse
import fcntl
import socket
import struct
import time
import redis

from proto.announcement_pb2 import Announcement
from software.py_constants import *

BROADCAST_INTERVAL_SECONDS = 2


def get_ip_address(interface: str) -> str:
    """Uses the Linux SIOCGIFADDR ioctl to find the IP address associated with a
    network interface, given the name of that interface

    :param interface: the interface to find the IP address associated with
    :return: the IP address associated with the given interface

    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(
        fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack("256s", bytes(interface, "utf-8")[:15]),
        )[20:24]
    )


def get_mac_address(interface: str) -> str:
    """Uses the Linux SIOCGIFHWADDR ioctl to find the HW/mac address associated
    with a network interface, given the name of that interface

    :param interface: the interface to find the HW/max address associated with
    :return: the HW/mac address associated with the given interface

    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    info = fcntl.ioctl(
        s.fileno(),
        0x8927,  # SIOCGIFHWADDR
        struct.pack("256s", bytes(interface, "utf-8")[:15]),
    )
    return ":".join("%02x" % b for b in info[18:24])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", required=True, type=int, help="port to send on")
    ap.add_argument(
        "-if",
        "--interface",
        required=True,
        type=str,
        help="interface to use to get ip and mac addr",
    )
    args = vars(ap.parse_args())

    redis_client = redis.Redis(host="localhost", port=6379, db=0)

    # Construct a announcement protobuf
    announcement = Announcement()
    announcement.robot_id = int(redis_client.get(ROBOT_ID_REDIS_KEY))
    announcement.ip_addr = get_ip_address(args["interface"])
    announcement.mac_addr = get_mac_address(args["interface"])

    # Send the announcement protobuf on the broadcast ip and specified port
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print("Starting broadcast..")
    while True:
        sender.sendto(announcement.SerializeToString(), ("<broadcast>", args["port"]))
        print("Sent announcement: ", announcement)
        time.sleep(BROADCAST_INTERVAL_SECONDS)


if __name__ == "__main__":
    main()
