#!/bin/bash

# Timeout in seconds
# When the time is up and no error was shown, this test will pass 
TIME_LIMIT=120 # 2 minutes

# Match Python traceback
ERROR_PATTERN="Traceback (most recent call last):"

# Temporary log file
LOG_FILE=$(mktemp)

# Run the command and record the log
"$@" &> "$LOG_FILE" &
CMD_PID=$!

# Time the process
SECONDS=0
while kill -0 $CMD_PID 2>/dev/null; do
    # Check if time is up
    if [ $SECONDS -ge $TIME_LIMIT ]; then
        echo "Time limit reached, stopping process: $CMD_PID"
        kill $CMD_PID
        wait $CMD_PID
        exit 0  # Upon time out and no error, returns 0 status code
    fi

    # Check if the log contains Traceback
    if grep -q "$ERROR_PATTERN" "$LOG_FILE"; then
	cat $LOG_FILE
        echo "[Error detected] Potential error found in command output!"
        kill $CMD_PID
        wait $CMD_PID
        exit 1
    fi

    sleep 1  # Run this loop once per second
done

# Get the exit code of the process
wait $CMD_PID
EXIT_CODE=$?

# Clean up log file
rm -f "$LOG_FILE"

# Exit with the command status code
exit $EXIT_CODE

 
