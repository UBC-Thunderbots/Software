#!/opt/tbotspython/bin/python3

"""
ci_runner.py runs a given command for a fix amount of time and
return 0 if there is no Python runtime exception found in the output
otherwise return 1

Because bazel outputs in both stderr and stdout regardless the content,
(https://github.com/bazelbuild/bazel/issues/10496)
only checking stderr is insufficient. Therefore, we are doing a pattern
matching in both stderr and stdout, which tells us if there is any runtime exception.
"""

from typing import List
import subprocess
import sys
import time
import select

# Default timeout is 120 seconds
TIME_LIMIT_S = 30
# Key pattern of python exception
PYTHON_ERROR_PATTERN="Traceback (most recent call last):"
# Delimiter which splits the target command output and this program logs
SECTION_DELIM = "=" * 50


def print_command(command: List[str]) -> None:
    """
    Format and print commands

    :param command: command to be printed in a list of string
    """
    print(" ".join(command))

def read_available_output(proc: subprocess.Popen) -> str:
    """
    Safely read available output from process without blocking

    :param proc: Running process
    :return: string output of stdout
    """
    output = ""
    if proc.stdout:
        rlist, _, _ = select.select([proc.stdout], [], [], 0)
        if rlist:
            output = proc.stdout.readline()
    return output

def test_command(command: List[str]) -> int:
    """
    Run a given command and return status code 

    :param command: command to run and test
    :return: 0 if the given command was run successfully and did not 
                throw any error in the given time limit.
             1 if the give command failed to run or threw errors to console.
    """
    start_time = time.time()

    # Run the command as a subprocess
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    
    # Keep polling and checking ONLY if
    # - did not reach time limit yet AND
    # - the process is still running
    while time.time() - start_time <= TIME_LIMIT_S and proc.poll() is None:
        stdout_data = read_available_output(proc)
        if PYTHON_ERROR_PATTERN in stdout_data:
            print(SECTION_DELIM)
            print("Oops! Error found while running the following command :(")
            print_command(command)

            proc.kill()
            return 1
        if stdout_data:
            print(stdout_data, end="")

    # If the process is still running, send SIGKILL signal
    if proc.poll() is None:
        # TODO: remove the following print statement
        print("killing the proc")
        proc.kill()
    
    remaining_output = proc.communicate()[0]
    print(remaining_output)
    print(SECTION_DELIM)

    print("Nice! Test passed! :)")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: command_runner.py <command> [args...]", file=sys.stderr)
        sys.exit(1)

    command = sys.argv[1:]
    print(f"Testing the following command:")
    print_command(command)

    print(SECTION_DELIM)
    sys.exit(test_command(sys.argv[1:]))

