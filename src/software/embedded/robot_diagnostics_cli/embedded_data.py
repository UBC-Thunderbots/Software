import redis
from software.py_constants import *
from proto.import_all_protos import *


class EmbeddedData:
    def __init__(self):
        self.redis = redis.StrictRedis(
            host=REDIS_DEFAULT_HOST,
            port=REDIS_DEFAULT_PORT,
            charset="utf-8",
            decode_responses=True
        )
    def get_robot_id(self):
        return int(self.redis.get(ROBOT_ID_REDIS_KEY))

    def get_network_interface(self):
        return self.redis.get(ROBOT_NETWORK_INTERFACE_REDIS_KEY)
