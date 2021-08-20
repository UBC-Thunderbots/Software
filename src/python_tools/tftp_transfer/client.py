import tftpy, argparse

PORT = 21

# parse command line arguments
ap      = argparse.ArgumentParser()
ap.add_argument("-p", "--port", required=True, type=int,
               help="inbound port")
ap.add_argument("-r", "--root_dir", required=True, type=str,
               help="root directory for the TFTP server")

args    = vars(ap.parse_args())
port    = args['port']

server = tftpy.TftpServer(args['root_dir'])
server.listen('0.0.0.0', PORT)