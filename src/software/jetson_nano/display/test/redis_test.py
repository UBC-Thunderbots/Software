import redis
import subprocess

def run():
    # connect to server
    r = redis.Redis(host='localhost', port=6379, db=0)

    # define some data structure to get values locally
    redis_dict = {}

    r.set('robot_id', 1)
    r.set('battery_level_1', 10)
    # redis is basically a database in RAM that holds keys and values like a map
    # get some values
    redis_dict['robot_id'] = r.get('robot_id').decode('UTF-8')
    redis_dict['battery_level_1'] = r.get('battery_level_1').decode('UTF-8')

    for key, val in redis_dict.items(): 
        print("{} : {}".format(key, val))

    # assign some value to key, or update a key
    r.set('battery_level_1', 80)
    redis_dict['battery_level_1'] = r.get('battery_level_1').decode('UTF-8')

    for key, val in redis_dict.items(): 
        print("{} : {}".format(key, val))


if __name__ == '__main__':
    # start redis server
    cmd = "sudo docker-compose up -d"
    st, out = subprocess.getstatusoutput(cmd)
    #print(out)
    run()

    # stop redis server
    cmd = "sudo docker-compose down"
    st, out = subprocess.getstatusoutput(cmd)

    #print(st)
    