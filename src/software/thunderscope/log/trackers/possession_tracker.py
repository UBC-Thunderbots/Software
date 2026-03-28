from software.thunderscope.log.trackers.tracker import Tracker
from typing import override, Callable
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from proto.import_all_protos import *
import software.python_bindings as tbots_cpp
from software.thunderscope.log.trackers.tracked_event import EventType


class PossessionTracker(Tracker):
    """Tracker to track and log when ball possession changes"""

    def __init__(self, proto_unix_io: ProtoUnixIO, out_file_path: str, buffer_size: int = 5):
        """Initializes the Possession tracker
             
        :param proto_unix_io: the proto unix io to get the game state from
        :param out_file_path: the file to write tracked events to               
        :param buffer_size: buffer size for the tracker's io
        """
        super().__init__(proto_unix_io, out_file_path, buffer_size)
        
        # start with no team having possession
        self.curr_possession = None
        
    @override
    def refresh(self):
        """Refresh and logs any changes in ball possession"""
        super().refresh()
        
        if self.cached_world is None:
            return
        
        self._log_posession_for_friendly(
            self.cached_world.friendlyTeam(), self.cached_world.enemyTeam(), self.cached_world.ball().position()
        )

    def _log_posession_for_friendly(
        self,
        friendly_team: tbots_cpp.Team,
        enemy_team: tbots_cpp.Team,
        ball_position: tbots_cpp.Point,
    ) -> None:
        """
        Detects possession changes and logs the corresponding start/end events
    
        Checks the current world state against the last known possession. 
        True if friendly possession, False is enemy possession, None if neither
        
        If a transition occurs (e.g., Friendly -> None, None -> Enemy), it triggers 
        a write_event with the appropriate EventType and updates the internal 
        possession state.

        :param friendly_team: the friendly team object
        :param enemy_team: the enemy team object
        :param ball_position: the current position of the ball
        :return: None
        """
        new_possession = None
        
        if self._check_posession_for_team(friendly_team, ball_position):
            new_possession = True
        elif self._check_posession_for_team(enemy_team, ball_position):
            new_possession = False
    
        # if possession didn't change, no need to log
        if new_possession == self.curr_possession:
            return
        
        event_type = None
        if new_possession:
            event_type = EventType.FRIENDLY_POSSESSION_START
        elif self.curr_possession:
            event_type = EventType.FRIENDLY_POSSESSION_END
        elif new_possession == False:
            event_type = EventType.ENEMY_POSSESSION_START
        else:
            event_type = EventType.ENEMY_POSSESSION_END
            
        self.write_event(event_type=event_type)
        
        self.curr_possession = new_possession

    def _check_posession_for_team(
        self, team: tbots_cpp.Team, ball_position: tbots_cpp.Point
    ) -> bool:
        """Check if the given team has possession of the ball

        :param team: the team to check
        :param ball_position: the current ball position
        :return: True if the team has possession, False otherwise
        """
        for robot in team.getAllRobots():
            if robot.isNearDribbler(ball_position):
                return True

        return False
