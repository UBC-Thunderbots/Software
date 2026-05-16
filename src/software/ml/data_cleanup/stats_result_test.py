import pytest
from unittest.mock import MagicMock, patch
from software.ml.data_cleanup.stats_result import StatsResult
from software.evaluation.logs.event_log import Team, EventType


class TestStatsResult:
    FRIENDLY_TEAM = Team.BLUE
    ENEMY_TEAM = Team.YELLOW

    @pytest.fixture
    def stats(self, friendly_team):
        # Initialize the dataclass
        return StatsResult(friendly_team=self.FRIENDLY_TEAM)

    def create_mock_event(self, event_type, for_team, ball_pos=(0, 0)):
        mock_event = MagicMock()
        mock_event.event_type = event_type
        mock_event.for_team = for_team
        mock_event.world_state_log.ball_state.get_position.return_value = ball_pos
        return mock_event

    def test_increment_passes(self, stats, friendly_team):
        event = self.create_mock_event(EventType.PASS, for_team=self.FRIENDLY_TEAM)

        stats.update_result(event)

        assert stats.passes == 1

        enemy_event = self.create_mock_event(EventType.PASS, for_team=self.ENEMY_TEAM)
        stats.update_result(enemy_event)
        assert stats.passes == 1  # Still 1

    def test_goal_scoring(self, stats, friendly_team):
        friendly_goal = self.create_mock_event(
            EventType.GOAL_SCORED, for_team=self.FRIENDLY_TEAM
        )
        enemy_goal = self.create_mock_event(
            EventType.GOAL_SCORED, for_team=self.ENEMY_TEAM
        )

        stats.update_result(friendly_goal)
        assert stats.score == 1
        assert stats.enemy_score == 0

        stats.update_result(enemy_goal)
        assert stats.score == 1
        assert stats.enemy_score == 1

    def test_possession(self, stats):
        stats.update_result(
            self.create_mock_event(EventType.FRIENDLY_POSSESSION_START, None)
        )
        assert stats.has_possession is True

        stats.update_result(
            self.create_mock_event(EventType.FRIENDLY_POSSESSION_END, None)
        )
        assert stats.has_possession is None

        stats.update_result(
            self.create_mock_event(EventType.ENEMY_POSSESSION_START, None)
        )
        assert stats.has_possession is False

    @patch.object(StatsResult, "_is_in_enemy_half")
    def test_ball_position_update(self, mock_is_in_half, stats):
        mock_is_in_half.return_value = True
        event = self.create_mock_event(EventType.PASS, self.FRIENDLY_TEAM)

        stats.update_result(event)

        assert stats.ball_in_enemy_half is True
        mock_is_in_half.assert_called_once()

    def test_game_reset(self, stats, friendly_team):
        stats.score = 5
        stats.passes = 10
        stats.has_possession = True

        reset_event = self.create_mock_event(EventType.GAME_START, friendly_team)
        stats.update_result(reset_event)

        assert stats.score == 0
        assert stats.passes == 0
        assert stats.has_possession is None
        assert stats.ball_in_enemy_half is False


if __name__ == "__main__":
    pytest.main([__file__])
