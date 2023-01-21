from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)


def test_method1():

    x = 0

    try:
        x = 5
        y = 10
        if x != y:
            raise AssertionError("not equal")
    except AssertionError:
        x = x + 1

    assert x == 1


if __name__ == "__main__":
    pytest_main(__file__)
