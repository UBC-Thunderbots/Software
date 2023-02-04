#!/bin/bash

cd "/opt/tbotspython/autoReferee/"
export JAVA_HOME="/usr/lib/jvm/jdk-17"

if [[ -z $1 ]];
then
    bin/autoReferee -a -hl
else
    bin/autoReferee -a -hl $1
fi

