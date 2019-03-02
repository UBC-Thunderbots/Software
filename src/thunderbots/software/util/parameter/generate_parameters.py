#!/usr/bin/env python
"""
This script runs as a preprocessor for catkin_make and
generates the necessary cfg and c++ files before compiling
to setup the DynamicParameters
"""
import yaml
import os
import time
import pprint


#######################################################################
#                               GLOBALS                               #
#######################################################################
PATH_TO_YAML = './config/'
PARAM_INFO = {}

def load_configuration(path_to_yaml):
    """Loads the yaml files in the current directory and
    makes a

    :param arg1: TODO
    :type arg1: TODO
    :returns: TODO
    :rtype: TODO

    """
    pass


# parse yaml and create parameter dictionaries
time.sleep(10)
print os.getcwd()
for filename in os.listdir(PATH_TO_YAML):
    if filename.endswith(".yaml"):
        with open(PATH_TO_YAML+filename, 'r') as param_yaml:
            PARAM_INFO[filename] = yaml.load(param_yaml)
            print PARAM_INFO
