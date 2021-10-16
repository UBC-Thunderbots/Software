import tftpy, hashlib, os, argparse, pathlib
from python_tools.robot_broadcast_receiver import receive_announcements


MAX_UPLOAD_RETRIES = 3


def encode_sha256_checksum(file_path: str) -> str:
    """
    Determines the SHA-256 checksum for a given file.
    
    :param file_path: the path to a file
    :return the SHA-256 checksum for the given file
    """
    # make hash by reading consecutive parts of data
    hash = hashlib.sha256()
    with open(file_path, "rb") as f:
        for byte_block in iter(lambda: f.read(4096), b""):
            hash.update(byte_block)

    return hash.hexdigest()


def main():
    # get command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-f",
        "--file",
        required=True,
        type=pathlib.Path,
        help="path to the file to be transferred. This should be an absolute path",
    )
    ap.add_argument(
        "--tftp_port",
        type=int,
        help="the port to be used for tftp transfers",
        required=True,
    )
    ap.add_argument(
        "--announce_port",
        type=int,
        help="the port to be used for listening to announcements",
        required=True,
    )
    ap.add_argument(
        "-d",
        "--duration",
        type=int,
        help="how long to listen for announcements. Recommended > 2",
        required=True,
    )
    ap.add_argument(
        "-r",
        "--retries",
        type=int,
        help="number of times to retry",
        default=MAX_UPLOAD_RETRIES,
        required=False,
    )

    args = vars(ap.parse_args())

    file_path = args["file"]
    file_name = os.path.basename(file_path)
    transfer_file_hash = encode_sha256_checksum(file_path)

    tftp_port = args["tftp_port"]
    announce_port = args["announce_port"]

    duration = args["duration"]
    max_retries = args["retries"]

    retries_count = 0
    # keep track of announcements from robots that have the correct sha256 checksum so that we dont re-upload files
    # Note: we use a map of robot_id to announcement because announcements are not hashable
    verified_announcements = {}
    clients = {}

    # additional + 1 is due to needing to verify announcements
    while retries_count < max_retries + 1:
        announcements = {
            announcement.robot_id: announcement
            for announcement in receive_announcements(announce_port, duration)
        }
        # don't upload file to robot that has correct sha256 checksum
        announcements_to_verify = {
            robot_id: announcement
            for robot_id, announcement in announcements.items()
            if robot_id not in verified_announcements
        }

        # exit early when there are no more announcements to verify
        if not announcements_to_verify:
            break

        for announcement in announcements_to_verify.values():
            if announcement.ip_addr not in clients:
                clients[announcement.ip_addr] = tftpy.TftpClient(
                    announcement.ip_addr, tftp_port
                )
            # send file to the robot
            clients[announcement.ip_addr].upload(file_name, file_path)

            # verify the checksum
            print(f"Verifying checksum for robot id: {announcement.robot_id}")
            if announcement.sha256_checksum == transfer_file_hash:
                print("Checksum verified")
                verified_announcements[announcement.robot_id] = announcement
            else:
                # if we can't validate the checksum, re-upload the file on the next try.
                print("Checksum verification failed")
                print("Retrying...")

        retries_count += 1
    if retries_count <= 0:
        print("Enter a valid number of retries")
    elif 0 < retries_count <= max_retries:
        print("Files successfully uploaded for all robots")
    elif retries_count > max_retries:
        print("Files not successfully uploaded for all robots.")

    if len(verified_announcements) > 0:
        print(f"Successful for robot_id: {sorted(verified_announcements.keys())} ")


if __name__ == "__main__":
    main()
