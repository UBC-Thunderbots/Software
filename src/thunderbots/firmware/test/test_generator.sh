#!/bin/bash

# generates the test.c file that then gets added to our firmware test executable. The idea behind using a generated
# test.c, which contains the main function for our unit tests, is so that we don't have merge conflicts in test.c
# when people try to add new tests.

cd $1/unit_tests
# list all the test files
FILES=$(ls | grep _test.c)

cd $1
# template file name
TEMPLATE=$1/template.c
# output test file name
OUTPUT=$1/test.c

# clear the output file 
echo "" > $OUTPUT

# write all of the includes for the test file
for i in $FILES; do
    # adds the include lines for each header to the file
    # ${i/\.c/.h} replaces the .c in the file name with a .h
    # example for i == file.c:
    #   #include file.h >> $OUTPUT
    echo '#include "'${i/\.c/.h}'"' >> $OUTPUT
done

# read the lines from the template
while IFS= read -r line
do 
    # see if we found the line where we need to fill
    # in the calls to test functions
    # the line we are looking for says "// INSERT_TESTS_HERE"
    match=$(echo $line | grep INSERT_TESTS_HERE)
    length=$(echo $match | wc -m)
    # if the length of the match was greater than one
    # then we need to write the functions into the file
    if [[ $((length)) > 1 ]]; then 
        # write a function for each test file
        # example for file_test.c:
        #   run_file_test(); >> $OUTPUT
        for i in $FILES; do
            echo "    run_${i/\.c/()};" >> $OUTPUT
        done
    else 
        # write the line from the template
        echo "$line" >> $OUTPUT
    fi
done < "$TEMPLATE"

