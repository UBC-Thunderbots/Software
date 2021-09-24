from proto.announcement_pb2 import Announcement
import socket
import argparse
from time import time

RECEIVE_TIMEOUT_SECONDS = 0.2
RECEIVE_DURATION_SECONDS = 4


def receive_announcements(port: int) -> [Announcement]:
    """
    Returns a list of Announcements, without duplicates received within a time window of 4s on a specified port
    :param port: the port to listen for announcements on
    :return: a list of Announcements, without duplicates
    """
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    receiver.settimeout(RECEIVE_TIMEOUT_SECONDS)
    receiver.bind(("", port))

    announcements = []
    timeout = time() + RECEIVE_DURATION_SECONDS
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
    args = vars(ap.parse_args())

    port = args["port"]

    announcements = receive_announcements(port)
    for announcement in announcements:
        print(
            f"robot_id: {announcement.robot_id} \nip_addr: {announcement.ip_addr} \nmac_addr: {announcement.mac_addr} \n"
        )


if __name__ == "__main__":
    main()
