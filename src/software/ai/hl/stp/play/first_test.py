from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)


def test_method1():

    j = 0

    while j != 5:
        try:
            x = 5
            y = 10
            if x != y:
                raise AssertionError("not equal")
        except AssertionError:
            print("this test failed")
            j = j + 1

    assert j == 5


if __name__ == "__main__":
    pytest_main(__file__)
