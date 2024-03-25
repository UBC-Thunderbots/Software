import logging

# ANSI escape codes can be found at this answer:
# https://stackoverflow.com/questions/4842424/list-of-ansi-color-escape-sequences

BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, GREY = range(8)


class ThunderscopeLoggingFormatter(logging.Formatter):
    reset_code = "\x1b[0m"
    color_prefix = "\x1b[3%d;"
    bold_code = "1m"
    flashing_code = "5m"
    log_format_string = (
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s (%(filename)s:%(lineno)d)"
    )

    FORMATS = {
        logging.FATAL: color_prefix % RED
        + flashing_code
        + log_format_string
        + reset_code,
        logging.CRITICAL: color_prefix % RED
        + bold_code
        + log_format_string
        + reset_code,
        logging.ERROR: color_prefix % RED + log_format_string + reset_code,
        logging.WARN: color_prefix % YELLOW + log_format_string + reset_code,
        logging.WARNING: color_prefix % MAGENTA + log_format_string + reset_code,
        logging.INFO: color_prefix % GREY + log_format_string + reset_code,
        logging.DEBUG: color_prefix % GREEN + log_format_string + reset_code,
    }

    def format(self, record):
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)
