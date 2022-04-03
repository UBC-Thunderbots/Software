import argparse

import tftpy


def main():
    # parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", type=int, help="listening port", required=True)
    ap.add_argument(
        "-r",
        "--root_dir",
        required=True,
        type=str,
        help="root directory for the TFTP server. This should be an absolute path",
    )

    args = vars(ap.parse_args())
    port = args["port"]

    server = tftpy.TftpServer(args["root_dir"])
    try:
        server.listen(listenport=port)
    except PermissionError:
        print(
            "Elevated privilege is required to open port "
            + str(port)
            + ". Retry with a port >= 1024."
        )


if __name__ == "__main__":
    main()
