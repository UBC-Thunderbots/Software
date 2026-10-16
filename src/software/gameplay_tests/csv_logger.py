import csv
import os
import time


class CsvLogger:
    """Logs named values as rows of a CSV file, alongside the time they were
    logged and the run they belong to.

    Rows are appended, so reruns and earlier invocations writing to the same
    file are kept and can be told apart by their run_start column.
    """

    def __init__(self, path, fields):
        """
        :param path: the CSV file to log to, its directory is created if needed
        :param fields: the names of the values to log, in column order
        """
        self.path = path
        self.fieldnames = ["timestamp", "run_start", *fields]
        self.file = None
        self.writer = None
        self.run_start = None

    def start_run(self):
        """Start logging a new run, opening the file if it isn't open yet"""
        if self.file is None:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            is_new_file = (
                not os.path.exists(self.path) or os.path.getsize(self.path) == 0
            )
            self.file = open(self.path, "a", newline="")
            self.writer = csv.DictWriter(
                self.file, fieldnames=self.fieldnames, restval=""
            )
            if is_new_file:
                self.writer.writeheader()
        self.run_start = time.strftime("%Y-%m-%d %H:%M:%S")

    def log(self, values):
        """Log a row of values stamped with the current time. Values that
        aren't given are left blank.

        :param values: dict of field name to value
        """
        if self.writer is None:
            return
        self.writer.writerow(
            {"timestamp": time.time(), "run_start": self.run_start, **values}
        )
        # Flush every row so nothing is lost if the test is killed
        self.file.flush()

    def close(self):
        if self.file is not None:
            self.file.close()
            self.file = None
            self.writer = None
