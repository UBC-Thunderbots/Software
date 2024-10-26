#!/bin/bash

rm -r ~/thunderbots_hashes/ &> /dev/null #clean up old files
mkdir ~/thunderbots_hashes

touch ~/thunderbots_hashes/thunderloop.hash
touch ~/thunderbots_hashes/thunderloop.date

: '
md5sum produces the MD5 hash of a provided file, followed by the name of the file seperated by a space
example output:

a60a083f11289d0904c9c4bf8fa59a59  thunderloop

using the awk command filters out only the first argument passed to it, in this case the MD5 hash
    '
hash=$(md5sum ~/thunderbots_binaries/thunderloop | awk '{ print $1}')
run_date=$(date '+%x %H:%M')

echo "$hash" > ~/thunderbots_hashes/thunderloop.hash
echo "$run_date" > ~/thunderbots_hashes/thunderloop.date
echo "$hash"