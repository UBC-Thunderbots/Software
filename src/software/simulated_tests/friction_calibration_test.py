import pytest

import software.python_bindings as tbots
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.geometry_pb2 import Point, Angle
from software.simulated_tests.data_logger import DataLogger
from software.simulated_tests.pytest_main import pytest_main
from proto.message_translation.tbots_protobuf import create_world_state, parse_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import *
from software.simulated_tests.field_tests.field_test_fixture import *
from software.simulated_tests.simulated_test_fixture import *
from software.simulated_tests.tbots_test_fixture import tbots_test_runner

from proto.import_all_protos import *
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# todos
# see actual shape of vel filtered through world
# see actual shape of vel directly from vision
# idea: find second derivative, filter 1/3rd smallest values and find point which best models sliding+rolling
# if too noisy / unreliable, find friction through passing btw robots


class BallVelocityLogger(DataLogger):
    def __init__(self, data_buffer, initial_kick_velocity):
        self.data_buffer = data_buffer
        self.n = 0
        self.initial_kick_velocity = initial_kick_velocity
        self.log_start_condition_achieved = False
        self.log_stop_condition_achieved = False


    def start_log(self, ball_speed, ball_position, robot_position):
        if self.log_start_condition_achieved == True:
            return True

        if (robot_position - ball_position).length() > 0.1 and ball_speed >= self.initial_kick_velocity:
            self.log_start_condition_achieved = True
            print("started")
            return True

        return False

    def stop_log(self, ball_speed, ball_position):

        if not self.log_start_condition_achieved:
            return False

        if self.log_stop_condition_achieved == True:
            return True

        field = tbots.Field.createSSLDivisionBField().fieldLines()
        if ball_speed < 0.01 or not tbots.contains(field, ball_position):
            self.log_stop_condition_achieved = True
            print("stopped")
            return True

        return False

    def log_data(self, world, time_elapsed_s):

        if len(world.friendly_team.team_robots) == 0:
            return

        self.n = self.n + 1
        ball_speed = tbots.Vector(world.ball.current_state.global_velocity.x_component_meters,world.ball.current_state.global_velocity.x_component_meters).length()
        ball_position = tbots.Point(world.ball.current_state.global_position.x_meters,world.ball.current_state.global_position.y_meters)
        epoch_timestamp = world.ball.timestamp.epoch_timestamp_seconds
        robot_position = tbots.Point(world.friendly_team.team_robots[0].current_state.global_position.x_meters, world.friendly_team.team_robots[0].current_state.global_position.y_meters)

        if self.start_log(ball_speed, ball_position, robot_position) and not self.stop_log(ball_speed, ball_position):
            self.data_buffer[epoch_timestamp] = ball_speed

        return


    def get_data(self):
        return self.data_buffer



def test_simulator_kick_ball(simulated_test_runner):
    print("in test")

    ball_initial_position = tbots.Point(-2.5, 0)
    kick_velocity = tbots.Vector(3, 0)

    rob_pos = ball_initial_position - (kick_velocity.normalize() * 0.1)

    # Setup Ball
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[rob_pos],
            ball_location=ball_initial_position,
            ball_velocity=tbots.Vector(0, 0),
        ),
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()

    kick_origin = Point(
        x_meters=ball_initial_position.x(), y_meters=ball_initial_position.y()
    )

    params.assigned_tactics[0].kick.CopyFrom(
        KickTactic(
            kick_origin=kick_origin,
            kick_direction=Angle(radians=0.0),
            kick_speed_meters_per_second=kick_velocity.length(),
        )
    )

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # expected ball position

    initial_v = kick_velocity.length()

    # Always Validation
    always_validation_sequence_set = []

    # Eventually Validation
    eventually_validation_sequence_set = []

    buffer = {}
    logger = BallVelocityLogger(buffer, initial_v)

    print("starting test")
    simulated_test_runner.run_test(
        test_timeout_s=4,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        data_loggers=[logger]
    )
    print("finished test")

    analyze_friction_data(data=buffer)

def analyze_friction_data(data):

    # generate fake data
    #x,y = generate_data()

    x = np.array(list(data.keys()))
    y = np.array(list(data.values()))

    #filter: start from largest recorded velocity
    # index_of_largest_vel = np.argmax(y)
    # print("largest speed achieved = ", y[index_of_largest_vel])
    #
    # x = x[index_of_largest_vel:]
    # y = y[index_of_largest_vel:]


    m1,m2,t_idx = find_best_fits(x,y)

    svgol = svgolay_d2(y)
    dxdy2 = get_d2(x,svgol)
    dxdy2_var = get_d2_var_groups(x,y,3)


    print(f"found m1, m2 = {m1}, {m2}")
    plt.plot(x, y,'b')
    plt.plot(x,dxdy2,'g')
    plt.plot(x, svgol,'c')
    #plt.plot(x,dxdy2_var)
    plt.axvline(x=x[t_idx], color='red')
    plt.show()


def find_best_fits(x,y):

    p_best = np.polyfit(x, y, deg=1, full=True)
    best_error = p_best[1]
    m1 = 0
    m2 = p_best[0][0]
    transition_idx = 0
    for i in range(2,len(x)):
        p1= np.polyfit(x[:i], y[:i], deg=1, full=True)
        p2= np.polyfit(x[i:], y[i:], deg=1, full=True)
        err = p1[1] + p2[1]

        if err < best_error:
            best_error = err
            m1 = p1[0][0]
            m2 = p2[0][0]
            transition_idx = i

    return m1,m2, transition_idx

def generate_data():

    num_samples = 150 #around 5 seconds of data
    x_sampling_period = 1.0/30.0 #30hz
    x_noise = 0.001
    x_stop = (num_samples)*x_sampling_period

    y_start = 5.0 #initial kick velocity
    y_noise = 0.1

    m1 = -3.432327 #sliding acceleration
    m2 = -0.5 #rolling acceleration
    transition_factor = 5.0/7.0
    transition_speed = y_start *transition_factor
    stop_value = 0.1

    #generate x values
    x = np.linspace(start=0.0, stop=x_stop, num=num_samples)
    noise_x = np.random.uniform(-x_noise, x_noise, size=(num_samples,))

    #generate y values
    transition_x = abs((y_start - transition_speed)/m1)
    idx_transition_x = int(transition_x / x_sampling_period) #also size of y1
    print("transition x, idx =", transition_x, idx_transition_x)

    y1 = x[:idx_transition_x] * m1 + y_start

    last_y1 = y1[-1]
    y2 = x[idx_transition_x:] * m2 + (last_y1-m2*x[idx_transition_x-1])

    y = np.append(y1, y2)
    noise_y = np.random.normal(0.0, y_noise, size=(num_samples,))

    #apply noise
    x = x + noise_x
    y = y + noise_y

    return x,y


def svgolay_d2(y):
    y_filt = savgol_filter(y, window_length=3, polyorder=2)
    return y_filt


#uses centered difference
def get_d2(x,y):
    dydx2=[]

    for i in range(len(x)):
        if i==0:
            dx=x[i:i+2]
            dy=y[i:i+2]
            order=1
        elif i==len(x)-1:
            dx=x[i-1:i+1]
            dy=y[i-1:i+1]
            order=1
        else:
            dx=x[i-1:i+2]
            dy=y[i-1:i+2]
            order=2
        z=np.polyfit(dx,dy,len(dx)-1)
        f=np.poly1d(z)
        df=np.polyder(f)
        dydx2.append(float(df(x[i])))
    dydx2=np.array(dydx2)
    return dydx2

def get_d2_var_groups(x,y, grouping=3):
    dydx2=[]

    #must be odd
    if grouping % 2 == 0:
        grouping += 1

    half = int(grouping/2)

    for i in range(len(x)):
        if i<half:
            diff = i - half
            dx=x[i:i+grouping]
            dy=y[i:i+grouping]
            order=2
        elif i>=len(x)-half:
            dx=x[i-grouping:i+1]
            dy=y[i-grouping:i+1]
            order=2
        else:
            dx=x[i-half:i+(half + 1)]
            dy=y[i-half:i+(half + 1)]
            order=2
        z=np.polyfit(dx,dy,order)
        f=np.poly1d(z)
        df=np.polyder(f)
        dydx2.append(float(df(x[i])))
    dydx2=np.array(dydx2)
    return dydx2


if __name__ == "__main__":
    pytest_main(__file__)

