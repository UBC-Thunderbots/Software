from pyqtgraph.Qt import QtCore, QtGui
from proto.import_all_protos import *
from enum import Enum, IntEnum


class ProtoUnixIOTypes(Enum):
    """
    Different keys for Proto Unix IOs used by Thunderscope
    """

    SIM = 1
    BLUE = 2
    YELLOW = 3
    DIAGNOSTICS = 4
    CURRENT = 5


class TabNames(str, Enum):
    """
    Different keys for tabs used in various Thunderscope views
    """

    BLUE = "BLUE"
    YELLOW = "YELLOW"
    DIAGNOSTICS = "DIAGNOSTICS"
    GAMECONTROLLER = "GAMECONTROLLER"

    def __str__(self):
        return str.__str__(self)


class ParamTypes(Enum):
    """
    Different types of parameters used by setup methods for Thunderscope widgets
    """

    BOOL = 1
    PROTO_UNIX_IO = 2
    STRING = 3
    INT = 4
    LIST = 5


class IndividualRobotMode(IntEnum):
    """
    Enum for the mode of input for an individual robot
    """

    NONE = 0
    MANUAL = 1
    AI = 2


LINE_WIDTH = 3
SPEED_LINE_WIDTH = 2
SPEED_SEGMENT_SCALE = 0.2

ROBOT_RADIUS = 25

BALL_HEIGHT_EFFECT_MULTIPLIER = 3

# Time (in seconds) to sleep / delay the loop iteration for different protos
# in robot communications
ROBOT_COMMUNICATIONS_TIMEOUT_S = 0.02

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
