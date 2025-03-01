from logging import Logger, basicConfig, getLogger, INFO


def create_logger(name, log_level: int = INFO) -> Logger:
    """Create a logger given the name of the logger

    :param name: The name of the logger
    :param log_level: The log level of the logger

    :return: A Logger
    """
    basicConfig(
        level=log_level,
        format="%(asctime)s - [%(levelname)s] - [%(threadName)s] - %(name)s - (%(filename)s).%(funcName)s(%(lineno)d) - %(message)s",
    )

    return getLogger(name)
