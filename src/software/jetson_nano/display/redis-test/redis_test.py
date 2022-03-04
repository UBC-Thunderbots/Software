import redis, json
import subprocess


def run():
    # connect to server
    r = redis.Redis(host="localhost", port=6379, db=0)

    # define some data structure to get values locally
    redis_dict = {}

    wheels = [
        {"front right": 1},
        {"front left": 2},
        {"back right": 3},
        {"back left": 4},
    ]

    chip_and_kick = {"chip": 1, "kick": 2}

    r.set("robot_id", 1)
    r.set("wheels", json.dumps(wheels))
    r.hmset("chip and kick", chip_and_kick)
    # r.set('enable wheel', True)
    # redis is basically a database in RAM that holds keys and values like a map
    # get some values
    redis_dict["robot_id"] = r.get("robot_id").decode("UTF-8")
    redis_dict["wheels"] = json.loads(r.get("wheels").decode("UTF-8"))
    redis_dict["chip and kick"] = r.hgetall(
        "chip and kick"
    )  # r.hmget('chip and kick', ['chip', 'kick'])
    # redis_dict['enable wheel'] = r.get('enable wheel').decode('UTF-8')

    for key, val in redis_dict.items():
        print("{} : {}".format(key, val))

    dict = {True: "true", False: "false"}
    print(dict[True])
    # assign some value to key, or update a key
    r.set("robot_id", 80)
    redis_dict["robot_id"] = r.get("robot_id").decode("UTF-8")

    for key, val in redis_dict.items():
        print("{} : {}".format(key, val))


if __name__ == "__main__":
    # start redis server
    cmd = "sudo docker-compose up -d"
    st, out = subprocess.getstatusoutput(cmd)
    # print(out)
    run()

    # stop redis server
    cmd = "sudo docker-compose down"
    st, out = subprocess.getstatusoutput(cmd)

    # print(st)
