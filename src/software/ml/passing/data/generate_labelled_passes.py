import os
import csv

from typing import List

from software.evaluation.logs.event_log import EventLog, Team
from software.evaluation.logs.pass_log import PassLog
from software.ml.passing.data.labelled_passes import LabelledPass, label_passes
from software.ml.passing.data.pass_result import generate_pass_results

dir_path = os.path.dirname(os.path.realpath(__file__))
ml_dir_path = os.path.dirname(dir_path)
datasets_path = os.path.join(ml_dir_path, "datasets")

def load_and_label_passes(
    pass_csv_file, event_csv_file, friendly_team
) -> List[LabelledPass]:
    pass_logs = []

    with open(
        os.path.join(datasets_path, pass_csv_file), mode="r", encoding="utf-8"
    ) as pass_csv:
        reader = csv.reader(pass_csv)

        for row in reader:
            pass_logs.append(PassLog.from_csv_row(iter(row)))

    print("Passes loaded!")

    event_logs = []
    with open(
        os.path.join(datasets_path, event_csv_file), mode="r", encoding="utf-8"
    ) as event_csv:
        reader = csv.reader(event_csv)

        for row in reader:
            event_logs.append(EventLog.from_csv_row(iter(row)))

    print("Events loaded!")

    pass_results = generate_pass_results(
        event_logs=event_logs[0:10000], pass_logs=pass_logs, friendly_team=friendly_team
    )

    print("Pass Results Generated!")

    labelled_passes = label_passes(pass_results)

    print("Passes labelled!")

    return labelled_passes

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(
            "Usage: python generate_labelled_passes.py <pass_logs_csv_file_path> <event_logs_csv_file_path>"
        )
        sys.exit(1)

    labelled_passes = load_and_label_data(sys.argv[1], sys.argv[2], Team.BLUE)
