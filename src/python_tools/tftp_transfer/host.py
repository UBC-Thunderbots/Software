import tftpy, hashlib, os, argparse, pathlib, shutil

TEMP = os.path.join(os.getcwd(), '.temp')

def verify_checksum(file_path: str, 
                    checksum: str) -> bool :
    try :
        file_hash = encode_sha256_checksum(file_path)
    except Exception :
        return False
    
    return file_hash == checksum

def encode_sha256_checksum(file_path: str) -> str:
    # make hash
    hash = hashlib.sha256()
    with open(file_path, "rb") as f:
        for byte_block in iter(lambda: f.read(4096), b"") :
            hash.update(byte_block)

    return hash.hexdigest()    

# get command line arguments
ap = argparse.ArgumentParser()
ap.add_argument("-f", required=True, type=pathlib.Path,
               help="path to the file to be transferred")
ap.add_argument("-p", required=True, type=int,
               help="outbound tftp port to be used", default=69)

ap.add_argument("-ip", "--ip_address", required=True, type=str,
               help="ip address of outbound connection")

args = vars(ap.parse_args())

file_path           = args['f']
file_name           = os.path.basename(args['f'])
transfer_file_hash  = encode_sha256_checksum(file_path)
port                = args['p']
ip_address          = args['ip_address']
client              = tftpy.TftpClient(ip_address, port)

try :
    os.mkdir(TEMP)
except FileExistsError :
    pass

# listen to announcements; for each robot, get info about ip address and reported sha_checksum
# get binaries and calculate check_sum
# compare checksums and upload if necessary

# for each robot reported in the announcement, compare readable_hash to reported hash
# if has isn't the same, upload new:
client.upload(file_name, file_path)
while True :
    print("Verifying checksum...")
    transferred_file_path = os.path.join(TEMP, file_name)
    client.download(file_name, transferred_file_path)

    if verify_checksum(transferred_file_path, transfer_file_hash) :
        print("Checksum verified")
        break
    else :
        print("Checksum verification failed")
        print("Retrying...")
        client.upload(file_name, file_path)
        
print("Cleaning up temp directory")
shutil.rmtree(TEMP)