import redis
import sys


def main():
    r = redis.Redis(host='localhost', port=6379, decode_responses=True)
    try:
        operation = sys.argv[1]
    except:
        print ("Missing action to do")
    if operation == "set":
            key = sys.argv[2]
            value = sys.argv[3]
            r.set(key,value)
    elif operation == "get":
            key = sys.argv[2]
            print(str(r.get(key)))
    else:
        print("Operation not supported")
    
if __name__ == "__main__":
    main()