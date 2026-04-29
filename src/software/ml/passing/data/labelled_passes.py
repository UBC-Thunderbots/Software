from dataclasses import dataclass
from software.evaluation.logs.pass_log import PassLog, PassLogType
from software.evaluation.logs.log_interface import IEvalLog
from software.ml.passing.data.pass_result import PassResult
from software.ml.data_cleanup.stats_result import StatsResult
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

    @classmethod
    def get_num_cols(cls) -> int:
        return 9

    def to_array(self) -> List[Any]:
        return [
            self.has_score_changed,
            self.has_enemy_score_changed,
            self.have_yellow_cards_changed,
            self.have_red_cards_changed,
            self.has_possession_changed,
            self.have_shots_on_net_changed,
            self.is_enemy_possession,
            self.has_ball_in_half_changed,
            self.is_ball_in_enemy_half,
        ]
  
    @staticmethod
    def from_csv_row(row_iter: Iterator[str], **kwargs) -> "Label":
        def _parse(val: str) -> bool:
            return val.strip().lower() == "true"
            
        return Label(
            has_score_changed=_parse(next(row_iter)),
            has_enemy_score_changed=_parse(next(row_iter)),
            have_yellow_cards_changed=_parse(next(row_iter)),
            have_red_cards_changed=_parse(next(row_iter)),
            has_possession_changed=_parse(next(row_iter)),
            have_shots_on_net_changed=_parse(next(row_iter)),
            is_enemy_possession=_parse(next(row_iter)),
            has_ball_in_half_changed=_parse(next(row_iter)),
            is_ball_in_enemy_half=_parse(next(row_iter)),
        )


@dataclass
class LabelledPass:
    pass_log: PassLog
    result: StatsResult
    labels: dict[PassLogType, Label]

    @classmethod
    def get_num_labels() -> int:
        # exclude the 0s interval label
        return len(PassLogType) - 1

    @classmethod
    def get_num_cols(cls) -> int:
        labels_count = Label.get_num_labels() * Label.get_num_cols()
        return PassLog.get_num_cols() + StatsResult.get_num_cols() + labels_count

    def to_array(self) -> List[Any]:
        arr = []
        arr.extend(self.pass_log.to_array())
        arr.extend(self.result.to_array())
       
        for log_type in PassLogType:
            if log_type == PassLogType.RESULT_0S:
                continue

            label = self.labels.get(log_type, Label()) # Default to empty label if missing
            arr.extend(label.to_array())
        return arr

    
def label_passes(pass_results: list[PassResult]) -> list[LabelledPass]:
    # Map Pass IDs to their 0s result (game performance at the moment of the pass) for quick lookup
    # {pass_id: StatsResult at t0}
    t0_baselines: dict[uuid.UUID, PassResult] = {}
    for result in pass_results:
        if result.pass_log.pass_log_type == PassLogType.RESULT_0S:
            t0_baselines[result.pass_log.pass_id] = result

    pass_labels: dict[uuid.UUID, dict[PassLogType, Label]] = {}

    # start with the t0 baselines as the previous interval state
    baselines = t0_baselines.copy()

    # assign labels to all of the other pass results from other intervals
    # comparing them to the previous interval's state
    for result in pass_results:
        pass_id = result.pass_log.pass_id
        log_type = result.pass_log.pass_log_type

        if log_type != PassLogType.RESULT_0S:
            # Look up the baseline for this specific pass
            baseline = baselines.get(pass_id)

            if not baseline:
                # Handle edge case where the previous interval log might be missing for an ID
                continue

            pass_result = result.result
            baseline_result = baseline.result

            # Calculate deltas relative to the 0s mark
            label = Label(
                has_score_changed=pass_result.score > baseline_result.score,
                has_enemy_score_changed=pass_result.enemy_score
                > baseline_result.enemy_score,
                have_yellow_cards_changed=pass_result.yellow_cards
                > baseline_result.yellow_cards,
                have_red_cards_changed=pass_result.red_cards
                > baseline_result.red_cards,
                has_possession_changed=pass_result.has_possession
                != baseline_result.has_possession,
                have_shots_on_net_changed=pass_result.shots_on_net
                > baseline_result.shots_on_net,
                # State at current interval
                is_enemy_possession=pass_result.has_possession is False,
                has_ball_in_half_changed=pass_result.ball_in_enemy_half
                != baseline_result.ball_in_enemy_half,
                is_ball_in_enemy_half=pass_result.ball_in_enemy_half,
            )

            if result.pass_log.pass_id not in pass_labels:
                pass_labels[result.pass_log.pass_id] = {}

            pass_labels[result.pass_log.pass_id][result.pass_log.pass_log_type] = label

            # update the baseline to the current interval's result for future intervals
            baselines[result.pass_log.pass_id] = result

    labelled_passes = []

    for pass_result in t0_baselines.values():
        labelled_passes.append(
            LabelledPass(
                pass_log=pass_result.pass_log,
                result=pass_result.result,
                labels=pass_labels[pass_result.pass_log.pass_id],
            )
        )

    return labelled_passes
