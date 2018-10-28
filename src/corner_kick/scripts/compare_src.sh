#!/bin/bash
CACHE=".checksum"

if [ ! -f $CACHE ]; then
    find src -type f -name "*.*" -exec md5sum {} + | awk '{print $1}' | sort | md5sum > $CACHE
    exit 1
else 
    OLDCHECKSUM=$(<$CACHE)
    NEWCHECKSUM=`find src -type f -name "*.*" -exec md5sum {} + | awk '{print $1}' | sort | md5sum`:0:31

    OLDCHECKSUM=${OLDCHECKSUM:0:31}
    NEWCHECKSUM=${NEWCHECKSUM:0:31}

    if [ "$OLDCHECKSUM" == "$NEWCHECKSUM" ]
    then
    exit 0
    else
    echo $NEWCHECKSUM > $CACHE
    exit 1
    fi
fi