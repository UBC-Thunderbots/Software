import csv
from pathlib import Path

from software.stats.logs.event_log import EventLog
from software.stats.analysis.metrics import format_report, generate_team_metrics
from software.thunderscope.constants import RuntimeManagerConstants

EVENTS_PATH = Path(
    RuntimeManagerConstants.RUNTIME_EVENTS_DIRECTORY_PATH,
    RuntimeManagerConstants.RUNTIME_EVENTS_FILE,
)


def load_events(path: str | Path = EVENTS_PATH) -> list[EventLog]:
    """Load event logs from a headerless CSV file.
    :param path: Path to the CSV file written by StatsLogger.
    :return: All successfully parsed event logs.
    """
    events = []

    with open(path, newline="") as events_file:
        for row in csv.reader(events_file):
            if not row:
                continue

            event = EventLog.from_csv_row(iter(row))
            if event is not None:
                events.append(event)

    return events


def main() -> None:
    """Print basic evaluation metrics from the runtime event log."""
    if not EVENTS_PATH.exists():
        raise SystemExit(
            f"No event log found at {EVENTS_PATH}. "
            "Run a simulation with --record_stats first."
        )

    metrics = generate_team_metrics(load_events())

    print(f"Evaluation metrics from {EVENTS_PATH}:")
    print(format_report(metrics))


if __name__ == "__main__":
    main()
