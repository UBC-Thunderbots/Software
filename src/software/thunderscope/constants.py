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

GAME_CONTROLLER_URL = "http://localhost:8081"

# Mapping between RobotStatus Error Codes and their dialog messages
ERROR_CODE_MESSAGES = {
    ErrorCode.LOW_CAP: "Low Cap",
    ErrorCode.LOW_BATTERY: "Low Battery",
    ErrorCode.HIGH_BOARD_TEMP: "High Board Temp",
    ErrorCode.DRIBBLER_MOTOR_HOT: "Dribbler Motor Hot",
}


def create_vision_pattern_lookup(color1, color2):
    """
    There is no pattern to this so we just have to create
    mapping from robot id to the four corners of the vision pattern

    robot-id: top-right, top-left, bottom-left, bottom-right

    https://robocup-ssl.github.io/ssl-rules/sslrules.html

    :param color1: first ID color
    :param color2: second ID color
    :return: the vision pattern lookup made up of the given colors
    """
    return {
        0: [color1, color1, color2, color1],
        1: [color1, color2, color2, color1],
        2: [color2, color2, color2, color1],
        3: [color2, color1, color2, color1],
        4: [color1, color1, color1, color2],
        5: [color1, color2, color1, color2],
        6: [color2, color2, color1, color2],
        7: [color2, color1, color1, color2],
        8: [color2, color2, color2, color2],
        9: [color1, color1, color1, color1],
        10: [color1, color1, color2, color2],
        11: [color2, color2, color1, color1],
        12: [color1, color2, color2, color2],
        13: [color1, color2, color1, color1],
        14: [color2, color1, color2, color2],
        15: [color2, color1, color1, color1],
    }


def rgb_to_bw(r, g, b):
    """
    Converts the given RGB color values into the corresponding black and white RGB values
    :param r: red value
    :param g: green value
    :param b: blue value
    :return: RGB tuple of the given color in black and white
    """
    rgb_val = 0.3 * r + 0.59 * g + 0.11 * b
    return rgb_val, rgb_val, rgb_val


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
    BW_ROBOT_MIDDLE_BLUE = QtGui.QColor(*rgb_to_bw(0, 0, 255))
    ROBOT_SPEED_SLOW_COLOR = "black"
    NAVIGATOR_PATH_COLOR = "green"
    NAVIGATOR_OBSTACLE_COLOR = "orange"

    VALIDATION_PASSED_COLOR = "g"
    VALIDATION_FAILED_COLOR = "r"

    # Colors for vision pattern
    PINK = QtGui.QColor(255, 0, 255)
    GREEN = QtGui.QColor(0, 255, 0)

    # Creates a default vision pattern lookup with the actual colors used on the robots
    VISION_PATTERN_LOOKUP = create_vision_pattern_lookup(PINK, GREEN)

    # Colors for black and white vision pattern
    BW_PINK = QtGui.QColor(*rgb_to_bw(255, 0, 255))
    BW_GREEN = QtGui.QColor(*rgb_to_bw(0, 255, 0))

    # Creates a black and white vision pattern to indicate a disconnected robot
    BW_VISION_PATTERN_LOOKUP = create_vision_pattern_lookup(BW_PINK, BW_GREEN)
