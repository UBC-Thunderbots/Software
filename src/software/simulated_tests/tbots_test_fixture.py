import pytest
from software.simulated_tests.pytest_main import load_command_line_arguments
from software.simulated_tests.simulated_test_fixture import simulated_test_initializer
from software.field_tests.field_test_fixture import (
    field_test_initializer,
)


@pytest.fixture
def tbots_test_runner():
    args = load_command_line_arguments()

    if args.run_field_test:
        runner_fixture = field_test_initializer()
    else:
        runner_fixture = simulated_test_initializer()

    yield next(runner_fixture)

    # teardown
    try:
        yield next(runner_fixture)
    except StopIteration:
        pass
