#!/bin/bash

# Start the redis server
redis-server ./redis-stable/redis.conf

# Compile the cpp file
g++ -std=c++17 -o ./cpp_redis ./cpp_redis.cpp PATH/redis-plus-plus/build/libredis++.a PATH/hiredis/libhiredis.a -pthread

# Tests
echo "=============================================="
echo "TESTING ADDING IN PYTHON AND READING IN PYTHON"
echo "=============================================="
SET_KEY1="First_test"
SET_VALUE1="First_value"
echo "SET $SET_KEY1 to $SET_VALUE1"
python3 ./python_redis.py set $SET_KEY1 $SET_VALUE1 
echo "GET $SET_KEY1"
python3 ./python_redis.py get $SET_KEY1 
SET_VALUE1="New_first_value"
echo "SET $SET_KEY1 to $SET_VALUE1"
python3 ./python_redis.py set $SET_KEY1 $SET_VALUE1 
echo "GET $SET_KEY1"
python3 ./python_redis.py get $SET_KEY1 

echo "                                              "
echo "=============================================="
echo "TESTING ADDING IN CPP AND READING IN CPP"
echo "=============================================="

SET_KEY2="Second_test"
SET_VALUE2="Second_value"
echo "SET $SET_KEY2 to $SET_VALUE2"
./cpp_redis set $SET_KEY2 $SET_VALUE2
echo "GET $SET_KEY2"
./cpp_redis get $SET_KEY2
SET_VALUE2="New_second_value"
echo "SET $SET_KEY2 to $SET_VALUE2"
./cpp_redis set $SET_KEY2 $SET_VALUE2
echo "GET $SET_KEY2"
./cpp_redis get $SET_KEY2

echo "                                              "
echo "=============================================="
echo "TESTING ADDING IN PYTHON AND READING IN CPP"
echo "=============================================="

SET_KEY3="Third_test"
SET_VALUE3="Third_value"
echo "SET $SET_KEY3 to $SET_VALUE3"
python3 ./python_redis.py set $SET_KEY3 $SET_VALUE3
echo "GET $SET_KEY3"
./cpp_redis get $SET_KEY3
SET_VALUE3="New_third_value"
echo "SET $SET_KEY3 to $SET_VALUE3"
python3 ./python_redis.py set $SET_KEY3 $SET_VALUE3
echo "GET $SET_KEY3"
./cpp_redis get $SET_KEY3

echo "                                              "
echo "=============================================="
echo "TESTING ADDING IN CPP AND READING IN PYTHON"
echo "=============================================="

SET_KEY4="Fourth_test"
SET_VALUE4="Fourth_value"
echo "SET $SET_KEY4 to $SET_VALUE4"
./cpp_redis set $SET_KEY4 $SET_VALUE4
echo "GET $SET_KEY4"
python3 ./python_redis.py get $SET_KEY4
SET_VALUE4="New_fourth_value"
echo "SET $SET_KEY4 to $SET_VALUE4"
./cpp_redis set $SET_KEY4 $SET_VALUE4
echo "GET $SET_KEY4"
python3 ./python_redis.py get $SET_KEY4

# Kill the redis server 
redis-cli shutdown