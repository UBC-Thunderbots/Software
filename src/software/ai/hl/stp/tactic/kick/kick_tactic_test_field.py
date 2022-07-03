import pytest

import software.python_bindings as tbots
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.pytest_main import pytest_main
from proto.message_translation.tbots_protobuf import create_world_state, parse_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import *
from software.simulated_tests.field_tests.field_test_fixture import *
from software.simulated_tests.simulated_test_fixture import *
from software.simulated_tests.tbots_test_fixture import tbots_test_runner
from software.simulated_tests.data_logger import DataLogger

ball_initial_position = tbots.Point(-2.5, 0)
kick_velocity = tbots.Vector(6, 0)

rob_pos = ball_initial_position - (kick_velocity.normalize() * 0.5)
from numpy import arange


class PassTimeLogger(DataLogger):
    def __init__(self, passer_id, receiver_id):
        self.data_buffer = {}
        self.n = 0
        self.passer_id = passer_id
        self.receiver_id = receiver_id

        self.last_sender_ts = None
        self.last_sender_pos = None
        self.first_receiver_ts = None
        self.first_receiver_pos = None


    def ball_in_dribbler(self, robot_position, ball_position):
        return (robot_position-ball_position).length() < 0.1

    def log_data(self, world, time_elapsed_s):

        if len(world.friendly_team.team_robots) == 0:
            return

        self.n = self.n + 1
        ball_speed = tbots.Vector(world.ball.current_state.global_velocity.x_component_meters,world.ball.current_state.global_velocity.x_component_meters).length()
        ball_position = tbots.Point(world.ball.current_state.global_position.x_meters,world.ball.current_state.global_position.y_meters)
        epoch_timestamp = world.ball.timestamp.epoch_timestamp_seconds
        receiver_position = tbots.Point(world.friendly_team.team_robots[self.receiver_id].current_state.global_position.x_meters, world.friendly_team.team_robots[self.receiver_id].current_state.global_position.y_meters)
        sender_position = tbots.Point(world.friendly_team.team_robots[self.passer_id].current_state.global_position.x_meters, world.friendly_team.team_robots[self.passer_id].current_state.global_position.y_meters)

        if self.ball_in_dribbler(sender_position, ball_position):
            self.last_sender_ts = epoch_timestamp
            self.last_sender_pos = sender_position

        if self.ball_in_dribbler(receiver_position, ball_position):
            self.first_receiver_ts = epoch_timestamp
            self.first_receiver_pos = receiver_position

        self.data_buffer = {'last_sender_ts':self.last_sender_ts, 'last_sender_pos':self.last_sender_pos, 'first_receiver_ts': self.first_receiver_ts, 'first_receiver_pos':self.first_receiver_pos}
        return

    def get_data(self):
        return self.data_buffer


def test_basic_kick(
    tbots_test_runner,
):
    print("IN TEST")
    ball_initial_position = tbots.Point(-2, 0)
    kick_velocity = tbots.Vector(4, 0)

    rob_pos = tbots.Point(-2.0, 0.0)
    rob_pos_p = Point(x_meters=rob_pos.x(), y_meters=rob_pos.y())

    # ybots, bbots, ball_initial_position, kick_velocity = parse_world_state(simulated_test_runner.initial_worldstate)
    # rob_pos = bbots[0]
    # tbots_test_runner.set_worldState(
    #     create_world_state(
    #         [],
    #         blue_robot_locations=[rob_pos],
    #         ball_location=ball_initial_position,
    #         ball_velocity=tbots.Vector(0, 0),
    #     ),
    # )


    test_positions = [(-2.0, -0.3), (-2.0, 0.3), (-1.0, 0.3), (-1.0, -0.3)]
    test_angles = [0,45,90,180,270,0]

    #for fixed pos
    x,y = (-1,0)

    for angle in test_angles:
        rob_pos_p = Point(x_meters=x, y_meters=y)

        logger.info(angle)
        move_tactic = MoveTactic()
        move_tactic.destination.CopyFrom(rob_pos_p)
        move_tactic.final_speed = 0.0
        move_tactic.dribbler_mode = DribblerMode.OFF
        move_tactic.final_orientation.CopyFrom(Angle(radians=angle))
        move_tactic.ball_collision_type = BallCollisionType.AVOID
        move_tactic.auto_chip_or_kick.CopyFrom(AutoChipOrKick(autokick_speed_m_per_s=0.0))
        move_tactic.max_allowed_speed_mode = MaxAllowedSpeedMode.PHYSICAL_LIMIT
        move_tactic.target_spin_rev_per_s = 0.0


        # Setup Tactic
        params = AssignedTacticPlayControlParams()
        kick_origin = Point(
            x_meters=ball_initial_position.x(), y_meters=ball_initial_position.y()
        )

        params.assigned_tactics[0].move.CopyFrom(
            move_tactic
        )

        # params.assigned_tactics[0].kick.CopyFrom(
        #     KickTactic(
        #         kick_origin=kick_origin,
        #         kick_direction=Angle(radians=0.0),
        #         kick_speed_meters_per_second=kick_velocity.length(),
        #     )
        # )

        tbots_test_runner.set_tactics(params, True)
        time.sleep(5)

    # Always Validation
    always_validation_sequence_set = []

    # Eventually Validation
    eventually_validation_sequence_set = [
        [BallEntersRegion([tbots.Field.createSSLDivisionBField().centerCircle()])]
    ]

    tbots_test_runner.run_test(
        test_timeout_s=5,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
    )


#generate test parameters
# test_parameters = []
# BALL_SPEED = 3
# STARTING_DISTANCE = 0.8
# DISTANCE_INCREMENT = 0.1
# ENDING_DISTANCE = 1.0
#
# test_results = []
#
# r1_pos = tbots.Point(-1*tbots.Field.createSSLDivisionBField().fieldLines().xLength()/2 + 0.2, 0.0)
# r2_pos = r1_pos+tbots.Vector(STARTING_DISTANCE,0)
#
# for idx, dist in enumerate(arange(STARTING_DISTANCE, ENDING_DISTANCE, DISTANCE_INCREMENT)):
#
#     if idx % 2 == 0:
#         passer_id_tmp = 0
#         receiver_id_tmp = 1
#         passer_pos_tmp = r1_pos
#         receiver_pos_tmp = r2_pos
#     else:
#         passer_id_tmp = 1
#         receiver_id_tmp = 0
#         passer_pos_tmp = r2_pos
#         receiver_pos_tmp = r1_pos
#
#     param_to_add = (passer_id_tmp, receiver_id_tmp, passer_pos_tmp, receiver_pos_tmp, BALL_SPEED)
#     print("adding ", param_to_add)
#     test_parameters.append((passer_id_tmp, receiver_id_tmp, passer_pos_tmp, receiver_pos_tmp, BALL_SPEED))
#
#     r2_pos = r2_pos + tbots.Vector(DISTANCE_INCREMENT,0)
#
#
# @pytest.mark.parametrize(
#     "passer_id,receiver_id,passer_pos,receiver_pos, pass_speed",
#     [*test_parameters])
# def test_kick_calibration(tbots_test_runner, passer_id, receiver_id, passer_pos, receiver_pos, pass_speed):
#
#     r1_id = passer_id
#     r2_id = receiver_id
#     r1_pos = passer_pos
#     r2_pos = receiver_pos
#     pass_start = r1_pos + ( (r2_pos - r1_pos.toVector()).toVector().normalize() * 0.2)
#     print(pass_start)
#
#     kick_origin_p = Point(
#         x_meters=pass_start.x(), y_meters=pass_start.y()
#     )
#
#     receive_point_p = Point(
#         x_meters=r2_pos.x(), y_meters=r2_pos.y()
#     )
#
#     pass_velocity = tbots.Vector(pass_speed,0)
#     test_pass_p = Pass(passer_point=kick_origin_p, receiver_point=receive_point_p, pass_speed_m_per_s=pass_velocity.length())
#
#     tbots_test_runner.set_worldState(
#         create_world_state(
#             [],
#             blue_robot_locations=[r1_pos, r2_pos],
#             ball_location=pass_start,
#             ball_velocity=tbots.Vector(0, 0),
#         ),
#     )
#
#     # Setup Tactic
#     params = AssignedTacticPlayControlParams()
#
#
#     params.assigned_tactics[r1_id].kick.CopyFrom(
#         KickTactic(
#             kick_origin=kick_origin_p,
#             kick_direction=Angle(radians=0.0),
#             kick_speed_meters_per_second=pass_velocity.length(),
#         )
#     )
#
#     params.assigned_tactics[r2_id].receiver.CopyFrom(
#         ReceiverTactic(
#             receiver_pass=test_pass_p,
#             disable_one_touch_shot=True,
#         )
#     )
#
#     tbots_test_runner.set_tactics(params, True)
#
#     # Always Validation
#     always_validation_sequence_set = []
#
#     # Eventually Validation
#     # eventually_validation_sequence_set = [
#     #     [BallEntersRegion([tbots.Field.createSSLDivisionBField().centerCircle()])]
#     # ]
#     eventually_validation_sequence_set = [
#         []
#     ]
#
#     logger = PassTimeLogger(r1_id, r2_id)
#
#     tbots_test_runner.run_test(
#         test_timeout_s=5,
#         eventually_validation_sequence_set=eventually_validation_sequence_set,
#         data_loggers=[logger],
#         always_validation_sequence_set=always_validation_sequence_set,
#     )
#
#     result = logger.get_data()
#     test_results.append(result)
#
# @pytest.fixture(scope='session', autouse=True)
# def kick_analysis():
#     yield
#     # Will be executed after the last test
#     for i in test_results:
#         print(i)


if __name__ == "__main__":
    print("ENTERED")
    pytest_main(__file__)
