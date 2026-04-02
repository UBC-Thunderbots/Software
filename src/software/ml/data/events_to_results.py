from software.ml.data.results import IResult
from software.evaluation.events.event import IEvent

def process_game_events(events: list[IEvent], initial_stats: IResult) -> IResult:
    """
    Sorts events by world_state timestamp and updates the stats result incrementally.
    """
    # 1. Sort the events in place by the nested timestamp to ensure chronological order
    events.sort(key=lambda e: e.get_timestamp())
    
    # 2. Iteratively update the stats result
    for event in events:
        initial_stats.update_result(event)
        
    return initial_stats