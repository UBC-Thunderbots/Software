from dataclasses import dataclass
from software.evaluation.logs.pass_log import PassLog, PassLogType
from software.ml.passing.pass_result import PassResult
from software.ml.data.stats_result import StatsResult
import uuid


@dataclass
class Label:
    has_score_changed: bool = False
    has_enemy_score_changed: bool = False
    have_yellow_cards_changed: bool = False
    have_red_cards_changed: bool = False
    has_possession_changed: bool = False
    have_shots_on_net_changed: bool = False
    is_enemy_possession: bool = False
    has_ball_in_half_changed: bool = False
    is_ball_in_enemy_half: bool = False


@dataclass
class LabelledPass:
    pass_log: PassLog
    result: StatsResult
    label: Label


def label_passes(pass_results: list[PassResult]) -> list[LabelledPass]:
    # 1. Map Pass IDs to their 0s result (game performance at the moment of the pass) for quick lookup
    # {pass_id: StatsResult at t0}
    t0_baselines: dict[uuid.UUID, StatsResult] = {}
    for result in pass_results:
        if result.pass_log.pass_log_type == PassLogType.RESULT_0S:
            t0_baselines[result.pass_log.pass_id] = result.result

    labelled_passes: list[LabelledPass] = []

    # 2. assign labels to all of the other pass results from other intervals
    # comparing them to their time 0 baseline
    for result in pass_results:
        pass_id = result.pass_log.pass_id
        log_type = result.pass_log.pass_log_type

        # If it's the t0_baseline itself, give it a "zeroed" or default label
        if log_type != PassLogType.RESULT_0S:
            # Look up the t0_baseline for this specific pass
            t0_baseline = t0_baselines.get(pass_id)

            if not t0_baseline:
                # Handle edge case where a 0s log might be missing for an ID
                continue

            pass_result = result.result

            # Calculate deltas relative to the 0s mark
            label = Label(
                has_score_changed=pass_result.score > t0_baseline.score,
                has_enemy_score_changed=pass_result.enemy_score
                > t0_baseline.enemy_score,
                have_yellow_cards_changed=pass_result.yellow_cards
                > t0_baseline.yellow_cards,
                have_red_cards_changed=pass_result.red_cards > t0_baseline.red_cards,
                has_possession_changed=pass_result.has_possession
                != t0_baseline.has_possession,
                have_shots_on_net_changed=pass_result.shots_on_net
                > t0_baseline.shots_on_net,
                # State at current interval
                is_enemy_possession=pass_result.has_possession is False,
                has_ball_in_half_changed=pass_result.ball_in_enemy_half
                != t0_baseline.ball_in_enemy_half,
                is_ball_in_enemy_half=pass_result.ball_in_enemy_half,
            )

        labelled_passes.append(
            LabelledPass(pass_log=result.pass_log, result=result.result, label=label)
        )

    return labelled_passes
