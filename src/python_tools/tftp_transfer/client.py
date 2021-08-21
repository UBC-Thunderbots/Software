import tftpy, argparse

# parse command line arguments
ap      = argparse.ArgumentParser()
ap.add_argument("-p", "--port", type=int, default=42000,
               help="inbound port")
ap.add_argument("-r", "--root_dir", required=True, type=str,
               help="root directory for the TFTP server")

args    = vars(ap.parse_args())
port    = args['port']

server = tftpy.TftpServer(args['root_dir'])
try :
    server.listen('0.0.0.0', port)
except PermissionError:
    print("Elevated privilege is required to use a port less than 1024. Retry with another port.")