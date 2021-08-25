import tftpy, hashlib, os, argparse, pathlib, logging

# TODO (#2237): following dependency isn't necessary when checksum is obtained from announcements
import tempfile


def verify_checksum(file_path: str, checksum: str) -> bool:
    """
    Compare the SHA-256 checksum of a given file and verify that it is the same as a given checksum.
    
    Any file-reading errors will return false.
    
    :param file_path: the path to a file
    :param checksum: the target SHA-256 checksum to compare against
    :return true if both checksums are correct, false otherwise
    """
    try:
        file_hash = encode_sha256_checksum(file_path)
    except Exception:
        logging.exception("Unexpected error when handling checksum verification.")
        return False

    return file_hash == checksum


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
        help="path to the file to be transferred",
    )

    # TODO (#2237): when announcements are working, we can just obtain the port and ip address from them and the
    #               following two arguments aren't necessary
    ap.add_argument("-p", "--port", type=int, help="the port to be used", required=True)
    ap.add_argument(
        "-ip",
        "--ip_address",
        required=True,
        type=str,
        help="ip address of outbound connection",
    )

    args = vars(ap.parse_args())
    file_path = args["file"]
    file_name = os.path.basename(args["file"])
    transfer_file_hash = encode_sha256_checksum(file_path)
    port = args["port"]
    ip_address = args["ip_address"]
    client = tftpy.TftpClient(ip_address, port)

    # send file to the robot
    client.upload(file_name, file_path)
    while True:
        # verify the checksum
        print("Verifying checksum...")
        # TODO (#2237): currently, the checksum is verified by redownloading the file that we just
        #       transferred from the robot to a temp directory & making sure the checksum
        #       of the redownloaded file is the same. When we have announcements, the
        #       checksum will be reported by the robot and we just compare the hash from the
        #       announcement
        with tempfile.TemporaryDirectory() as tmpdirname:
            transferred_file_path = os.path.join(tmpdirname, file_name)
            client.download(file_name, transferred_file_path)

            if verify_checksum(transferred_file_path, transfer_file_hash):
                print("Checksum verified")
                break
            else:
                # if we can't validate the checksum, re-upload the file.
                print("Checksum verification failed")
                print("Retrying...")
                client.upload(file_name, file_path)


if __name__ == "__main__":
    main()
