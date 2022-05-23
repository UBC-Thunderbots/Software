import argparse
import socket
from time import time

from proto.announcement_pb2 import Announcement

RECEIVE_TIMEOUT_SECONDS = 0.2


def receive_announcements(port: int, duration: int) -> [Announcement]:
    """Returns a list of Announcements, without duplicates received within a
    time window of 4s on a specified port

    :param port: the port to listen for announcements on
    :param duration: how long to listen for announcements
    :return: a list of Announcements, without duplicates

    """
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    receiver.settimeout(RECEIVE_TIMEOUT_SECONDS)
    receiver.bind(("", port))

    announcements = []
    timeout = time() + duration
    while time() < timeout:
        try:
            data = receiver.recv(1024)
        except socket.timeout:  # ignore timeout errors
            continue
        else:
            # parse announcement protobuf
            announcement = Announcement()
            announcement.ParseFromString(data)
            # filter out duplicates
            if announcement not in announcements:
                announcements.append(announcement)
    return announcements


def main():
    # get command line args
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", required=True, type=int, help="port to listen on")
    ap.add_argument(
        "-d",
        "--duration",
        required=True,
        type=int,
        help="how long to listen for announcements. Recommended > 2",
    )
    args = vars(ap.parse_args())
    port = args["port"]
    duration = args["duration"]

    announcements = receive_announcements(port, duration)
    for announcement in announcements:
        print(
            f"robot_id: {announcement.robot_id} \nip_addr: {announcement.ip_addr} \nmac_addr: {announcement.mac_addr} \n"
        )


if __name__ == "__main__":
    main()
