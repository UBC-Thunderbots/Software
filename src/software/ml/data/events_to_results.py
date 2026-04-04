from software.ml.data.result_interface import IResult
from software.evaluation.logs.log_interface import TimestampedEvalLog


def process_game_eval_logs(
    logs: list[TimestampedEvalLog], initial_stats: IResult
) -> IResult:
    """Sorts events by world_state timestamp and updates the stats result incrementally."""
    # 1. Sort the events in place by the nested timestamp to ensure chronological order
    logs.sort(key=lambda e: e.get_timestamp())

    # 2. Iteratively update the stats result
    for log in logs:
        initial_stats.update_result(log)

    return initial_stats
