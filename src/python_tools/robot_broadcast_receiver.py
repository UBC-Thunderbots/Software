from announcement_pb2 import Announcement
import socket
import errno
from time import time, sleep

receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receiver.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
receiver.setblocking(False)
receiver.bind(("", 42020))

announcements = []
timeout = time() + 4  # 4s

while time() < timeout:
    try:
        data = receiver.recv(1024)
    except socket.error as e:
        err = e.args[0]
        if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
            sleep(0.2)
            continue
    else:
        announcement = Announcement()
        announcement.ParseFromString(data)
        if announcement not in announcements:
            announcements.append(announcement)

for announcement in announcements:
    print(
        f"robot_id: {announcement.robot_id} \nip_addr: {announcement.ip_addr} \nmac_addr: {announcement.mac_addr} \n"
    )
