#!/bin/sh

cd "/opt/tbotspython/autoReferee/"
export JAVA_HOME="/usr/lib/jvm/jdk-17"
echo $JAVA_HOME
bin/autoReferee -a -hl --ci

