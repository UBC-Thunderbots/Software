from collections import Counter
from collections.abc import Iterable
from dataclasses import dataclass

from proto.ssl_gc_common_pb2 import Team as SslTeam
from software.stats.logs.event_log import EventLog, EventType

TEAMS = (SslTeam.BLUE, SslTeam.YELLOW)


@dataclass
class TeamMetrics:
    """Evaluation metrics for a single team over a set of events"""

    goals_scored: int = 0
    goals_conceded: int = 0
    shots_on_goal: int = 0
    shots_faced: int = 0
    shots_stopped: int = 0
    possession_ns: float = 0.0
    yellow_cards: int = 0
    red_cards: int = 0


def possession_time_ns(events: Iterable[EventLog]) -> float:
    """Total time the logging team had the ball, from its own possession events

    :param events: events written by a single team's StatsLogger
    :return: total possession time in nanoseconds
    """
    total = 0.0
    start = None
    last_timestamp = None

    for event in sorted(events, key=lambda event: event.timestamp):
        if event.event_type == EventType.FRIENDLY_POSSESSION_START:
            start = event.timestamp
        elif (
            event.event_type == EventType.FRIENDLY_POSSESSION_END and start is not None
        ):
            total += event.timestamp - start
            start = None

        last_timestamp = event.timestamp

    # the team still had the ball when the log ended
    if start is not None:
        total += last_timestamp - start

    return total


def generate_team_metrics(events: Iterable[EventLog]) -> dict[SslTeam, TeamMetrics]:
    """Generate evaluation metrics for each team

    Each team's StatsLogger only sees its own side of the game, so some metrics
    (e.g. goals conceded) come from the opponent's events.

    :param events: events written by both teams' StatsLoggers
    :return: metrics for each team, keyed by team
    """
    # only keep events a team logged about itself; events about the opponent
    # would double count what the opponent's own logger already recorded
    events = [event for event in events if event.from_team == event.for_team]
    counts = Counter((event.from_team, event.event_type) for event in events)

    metrics = {}
    for team, opponent in zip(TEAMS, reversed(TEAMS)):
        goals_conceded = counts[opponent, EventType.GOAL_SCORED]

        metrics[team] = TeamMetrics(
            goals_scored=counts[team, EventType.GOAL_SCORED],
            goals_conceded=goals_conceded,
            shots_on_goal=counts[opponent, EventType.ENEMY_SHOT_ON_GOAL],
            shots_faced=counts[team, EventType.ENEMY_SHOT_ON_GOAL],
            # SHOT_BLOCKED fires whenever an incoming shot ends, including goals
            shots_stopped=max(counts[team, EventType.SHOT_BLOCKED] - goals_conceded, 0),
            possession_ns=possession_time_ns(
                event for event in events if event.from_team == team
            ),
            yellow_cards=counts[team, EventType.YELLOW_CARD],
            red_cards=counts[team, EventType.RED_CARD],
        )

    return metrics


def _percent(numerator: float, denominator: float) -> str:
    """Formats a ratio as a percentage, or a dash if it is undefined"""
    return f"{numerator / denominator:.0%}" if denominator else "-"


def format_report(metrics: dict[SslTeam, TeamMetrics], column_width: int = 10) -> str:
    """Formats the team metrics as a table with a column per team

    :param metrics: metrics for each team, from generate_team_metrics()
    :param column_width: width of each team's column
    :return: the formatted table
    """
    total_possession_ns = sum(team.possession_ns for team in metrics.values())

    rows = [
        ("Goals scored", lambda team: team.goals_scored),
        ("Goals conceded", lambda team: team.goals_conceded),
        ("Shots on goal", lambda team: team.shots_on_goal),
        ("Save rate", lambda team: _percent(team.shots_stopped, team.shots_faced)),
        (
            "Possession",
            lambda team: _percent(team.possession_ns, total_possession_ns),
        ),
        ("Yellow cards", lambda team: team.yellow_cards),
        ("Red cards", lambda team: team.red_cards),
    ]

    label_width = max(len(label) for label, _ in rows)
    lines = [
        " " * label_width
        + "".join(f"{SslTeam.Name(team):>{column_width}}" for team in metrics)
    ]
    for label, value in rows:
        lines.append(
            label.ljust(label_width)
            + "".join(f"{value(team)!s:>{column_width}}" for team in metrics.values())
        )

    return "\n".join(lines)
