from shared.proto.announcement_pb2 import Announcement
import argparse
import fcntl
import socket
import struct
import os
import sys
from time import sleep

BROADCAST_INTERVAL_SECONDS = 2


def get_ip_address(ifname: str) -> str:
    """
    Uses the Linux SIOCGIFADDR ioctl to find the IP address associated with a network interface, given the name of that
    interface
    :param ifname: the interface to find the IP address associated with
    :return: the IP address associated with the given interface
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(
        fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack("256s", bytes(ifname, "utf-8")[:15]),
        )[20:24]
    )


def get_mac_address(ifname: str) -> str:
    """
    Uses the Linux SIOCGIFHWADDR ioctl to find the HW/mac address associated with a network interface, given the name of
    that interface
    :param ifname: the interface to find the HW/max address associated with
    :return: the HW/mac address associated with the given interface
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    info = fcntl.ioctl(
        s.fileno(),
        0x8927,  # SIOCGIFHWADDR
        struct.pack("256s", bytes(ifname, "utf-8")[:15]),
    )
    return ":".join("%02x" % b for b in info[18:24])


def main():
    # get command line args
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", required=True, type=int, help="port to send on")
    ap.add_argument(
        "-if",
        "--ifname",
        required=True,
        type=str,
        help="interface to use to get ip and mac addr",
    )
    ap.add_argument(
        "-q",
        "--quiet",
        required=False,
        default=False,
        type=bool,
        help="whether to print to terminal or not",
    )
    args = vars(ap.parse_args())

    interface = args["ifname"]
    port = args["port"]
    quiet = args["quiet"]

    # silence printing by piping stdout and stderr to dev/null
    if quiet:
        sys.stdout = open(os.devnull, "a")
        sys.stderr = open(os.devnull, "a")

    # construct a announcement protobuf
    announcement = Announcement()
    announcement.robot_id = 1  # TODO (#2229): read this value from the key-value store
    announcement.ip_addr = get_ip_address(interface)
    announcement.mac_addr = get_mac_address(interface)
    announcement.sha256_checksum = (
        ""  # TODO (#2229): read this value from the key-value store
    )

    # send the announcement protobuf on the broadcast ip and specified port
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print("Starting broadcast..")
    while True:
        sender.sendto(announcement.SerializeToString(), ("<broadcast>", port))
        print(announcement)
        sleep(BROADCAST_INTERVAL_SECONDS)


if __name__ == "__main__":
    main()
