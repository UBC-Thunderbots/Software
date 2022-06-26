import software.python_bindings as tbots
from proto.import_all_protos import *
from software.py_constants import *
import math

"""Helper functions for robot_speed_threshold.py and ball_speed_threshold.py"""

class SpeedThreshold:
    
    def __init__(self):

    def get_validation_line_endpoints(validation_centre_x, validation_centre_y, ball_angle):
        start_x = (
            validation_centre_x
            - math.sin(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )
        end_x = (
            validation_centre_x
            + math.sin(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )

        start_y = (
            validation_centre_y
            + math.cos(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )
        end_y = (
            validation_centre_y
            - math.cos(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )

        if ball_angle > math.pi or ball_angle < 0:
            start_y = (
                validation_centre_y
                + math.cos(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
            )
            end_y = (
                validation_centre_y
                - math.cos(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
            )

        return [start_x, end_x, start_y, end_y]
