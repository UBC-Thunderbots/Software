from logging import Logger, basicConfig, getLogger, INFO

basicConfig(
    level=INFO,
    format="%(asctime)s - [%(levelname)s] - [%(threadName)s] - %(name)s - (%(filename)s).%(funcName)s(%(lineno)d) - %(message)s",
)


def create_logger(name) -> Logger:
    """Create a logger given the name of the logger

    :returns: A Logger

    """
    return getLogger(name)
