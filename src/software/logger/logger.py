import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - [%(levelname)s] - [%(threadName)s] - %(name)s - (%(filename)s).%(funcName)s(%(lineno)d) - %(message)s",
)


def createLogger(name):
    """Create a logger given the name of the logger

    :returns: A Logger

    """
    return logging.getLogger(name)
