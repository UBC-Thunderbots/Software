#!/bin/bash

# Runs the Tigers Autoref binary that is set up by "setup_software.sh". By default, wrapper runs the autoreferee in headless, active mode. If an additional argument is passed in to the script, it is forwarded to the binary.

cd "/opt/tbotspython/autoReferee/"
export JAVA_HOME="/opt/tbotspython/bin/jdk"

bin/autoReferee -a $@

