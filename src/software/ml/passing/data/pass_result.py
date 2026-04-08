from dataclasses import dataclass, replace
from software.ml.data_cleanup.result_interface import IResult
from software.ml.data_cleanup.stats_result import StatsResult
from software.evaluation.logs.pass_log import PassLog
from software.evaluation.logs.event_log import EventLog, Team
from typing import List


@dataclass
class PassResult(IResult):
    pass_log: PassLog
    result: StatsResult


def generate_pass_results(
    event_logs: List[EventLog], pass_logs: List[PassLog], friendly_team: Team
) -> List[PassResult]:
    """Processes a mixed list of logs chronologically to map the
    game state (StatsResult) to specific PassLogs.
    """
    all_logs = event_logs + pass_logs

    # 1. Sort everything by timestamp.
    all_logs.sort(key=lambda x: x.timestamp)

    # 2. Initialize our running state
    current_stats = StatsResult(friendly_team=friendly_team)
    pass_results: List[PassResult] = []

    # 3. Iterate through the sorted logs
    for log in all_logs:
        if isinstance(log, EventLog):
            # Update the cumulative stats based on the EventLog
            current_stats.update_result(log)

        elif isinstance(log, PassLog):
            # Create a snapshot of the stats AT THIS MOMENT.
            snapshot = replace(current_stats)

            pass_results.append(PassResult(pass_log=log, result=snapshot))

    return pass_results
