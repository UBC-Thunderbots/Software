from announcement_pb2 import Announcement
import socket
import fcntl
import struct
import time


def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(
        fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack("256s", bytes(ifname, "utf-8")[:15]),
        )[20:24]
    )


def get_mac_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    info = fcntl.ioctl(
        s.fileno(), 0x8927, struct.pack("256s", bytes(ifname, "utf-8")[:15])
    )
    return ":".join("%02x" % b for b in info[18:24])


interface = "eth0"

announcement = Announcement()
announcement.robot_id = 1
announcement.ip_addr = get_ip_address(interface)
announcement.mac_addr = get_mac_address(interface)

sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sender.bind(("", 42021))

while True:
    sender.sendto(announcement.SerializeToString(), ("<broadcast>", 42020))
    print(announcement)
    time.sleep(2)
