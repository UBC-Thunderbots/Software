#!/bin/bash
#
# This script checks if the src folder has been modified by comparing the current
# MD5 checksum of the directory with the one stored from the last run of this script.
#
# If it has changed, or there are no previous value, the script returns with an error,
# allowing you to act on it. The script will also store the new value.


# Location to save the checksum value
CACHE=".checksum"

# Location of the src directory
SRC="src"

# File to include in the md5 checksum generation.
# By default, we include all files.
FILETYPE="*.*"

# Check if checksum file exists.
if [ ! -f $CACHE ]; then
    # File does not exist, assume src is dirty and save the checksum.
    find "$SRC" -type f -name "$FILETYPE" -exec md5sum {} + | awk '{print $1}' | sort | md5sum > $CACHE
    exit 1
else 
    # File exists, get the old value and generate the new value.
    OLDCHECKSUM=$(<$CACHE)
    NEWCHECKSUM=$(find "$SRC" -type f -name "$FILETYPE" -exec md5sum {} + | awk '{print $1}' | sort | md5sum)

    # The actual checksum is 32 characters long, 
    # so only get that portion of the string.
    OLDCHECKSUM=${OLDCHECKSUM:0:31}
    NEWCHECKSUM=${NEWCHECKSUM:0:31}

    # Compare the two checksum
    if [ "$OLDCHECKSUM" == "$NEWCHECKSUM" ]
    then
        # They are the same! No need to do anything.
        exit 0
    else
        # They are different, save the new checksum and return an error.
        echo $NEWCHECKSUM > $CACHE
        exit 1
    fi
fi
