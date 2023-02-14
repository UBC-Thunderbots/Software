from pyqtgraph.Qt import QtCore, QtGui
from proto.import_all_protos import *

LINE_WIDTH = 3
SPEED_LINE_WIDTH = 2
SPEED_SEGMENT_SCALE = 0.2

ROBOT_RADIUS = 25

# Mapping between RobotStatus Error Codes and their dialog messages
ERROR_CODE_MESSAGES = {
    ErrorCode.LOW_CAP: "Low Cap",
    ErrorCode.LOW_BATTERY: "Low Battery",
    ErrorCode.HIGH_BOARD_TEMP: "High Board Temp",
    ErrorCode.DRIBBLER_MOTOR_HOT: "Dribbler Motor Hot",
}


class Colors(object):

    FIELD_COLOR = "w"
    FIELD_LINE_COLOR = "w"

    BLUE_ROBOT = QtGui.QColor(255, 100, 0, 255)
    BALL_COLOR = QtGui.QColor(255, 100, 0, 255)
    SIM_BALL_COLOR = QtGui.QColor(255, 100, 0, 150)
    YELLOW_ROBOT_COLOR = QtGui.QColor(255, 255, 0, 255)
    BLUE_ROBOT_COLOR = QtGui.QColor(0, 75, 255, 255)
    TRANSPARENT = QtGui.QColor(0, 0, 0, 0)
    DESIRED_ROBOT_LOCATION_OUTLINE = QtGui.QColor(255, 0, 0, 150)
    SPEED_VECTOR_COLOR = QtGui.QColor(255, 0, 255, 100)

    ROBOT_MIDDLE_BLUE = "blue"
    ROBOT_SPEED_SLOW_COLOR = "black"
    NAVIGATOR_PATH_COLOR = "green"
    NAVIGATOR_OBSTACLE_COLOR = "orange"

    VALIDATION_PASSED_COLOR = "g"
    VALIDATION_FAILED_COLOR = "r"

    # Colors for vision pattern
    PINK = QtGui.QColor(255, 0, 255)
    GREEN = QtGui.QColor(0, 255, 0)

    # There is no pattern to this so we just have to create
    # mapping from robot id to the four corners of the vision pattern
    #
    # robot-id: top-right, top-left, bottom-left, bottom-right
    #
    # https://robocup-ssl.github.io/ssl-rules/sslrules.html
    VISION_PATTERN_LOOKUP = {
        0: [PINK, PINK, GREEN, PINK],
        1: [PINK, GREEN, GREEN, PINK],
        2: [GREEN, GREEN, GREEN, PINK],
        3: [GREEN, PINK, GREEN, PINK],
        4: [PINK, PINK, PINK, GREEN],
        5: [PINK, GREEN, PINK, GREEN],
        6: [GREEN, GREEN, PINK, GREEN],
        7: [GREEN, PINK, PINK, GREEN],
        8: [GREEN, GREEN, GREEN, GREEN],
        9: [PINK, PINK, PINK, PINK],
        10: [PINK, PINK, GREEN, GREEN],
        11: [GREEN, GREEN, PINK, PINK],
        12: [PINK, GREEN, GREEN, GREEN],
        13: [PINK, GREEN, PINK, PINK],
        14: [GREEN, PINK, GREEN, GREEN],
        15: [GREEN, PINK, PINK, PINK],
    }
