import pytest
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
import software.python_bindings as tbots

def test_drive_in_straight_line_setup(
        initial_position,
        friendly_robots,
        enemy_robots
):
    destination = tbots.Point(2.8, 0)
    


def test_drive_in_straight_line():



if __name__ == "__main__":
    pytest_main(__file__)