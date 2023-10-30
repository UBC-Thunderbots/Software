from pyqtgraph.Qt import QtCore, QtGui
from proto.import_all_protos import *
from enum import Enum, IntEnum

import textwrap


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


class CameraView(Enum):
    """
    Enum for preset camera views in the 3D visualizer
    """

    ORTHOGRAPHIC = 1
    LANDSCAPE_HIGH_ANGLE = 2
    LEFT_HALF_HIGH_ANGLE = 3
    RIGHT_HALF_HIGH_ANGLE = 4


class EstopMode(IntEnum):
    """
    Enum for the various estop modes we can run thunderscope in

    DISABLE_ESTOP: No physical / keyboard estop is needed, but we cannot send anything over the network
    KEYBOARD_ESTOP: The spacebar can be used as an estop toggle instead of a physical estop
    ESTOP: A physical estop is needed to run thunderscope, throws an exception if none is plugged in
    """

    DISABLE_ESTOP = 0
    KEYBOARD_ESTOP = 1
    PHYSICAL_ESTOP = 2


LINE_WIDTH = 3
SPEED_LINE_WIDTH = 2
SPEED_SEGMENT_SCALE = 0.2

DEFAULT_EMPTY_FIELD_WORLD = World(
    field=Field(
        field_x_length=9.0,
        field_y_length=6.0,
        defense_x_length=1.0,
        defense_y_length=2.0,
        goal_x_length=0.18,
        goal_y_length=1.0,
        boundary_buffer_size=0.3,
        center_circle_radius=0.5,
    )
)

ROBOT_RADIUS = 25

BALL_HEIGHT_EFFECT_MULTIPLIER = 3

# Time (in seconds) to sleep / delay the loop iteration for different protos
# in robot communications
ROBOT_COMMUNICATIONS_TIMEOUT_S = 0.02

GAME_CONTROLLER_URL = "http://localhost:8081"

ESTOP_PATH_1 = "/dev/ttyACM0"
ESTOP_PATH_2 = "/dev/ttyUSB0"

# Mapping between RobotStatus Error Codes and their dialog messages
ERROR_CODE_MESSAGES = {
    ErrorCode.LOW_CAP: "Low Cap",
    ErrorCode.LOW_BATTERY: "Low Battery",
    ErrorCode.HIGH_BOARD_TEMP: "High Board Temp",
    ErrorCode.DRIBBLER_MOTOR_HOT: "Dribbler Motor Hot",
}

SAVED_LAYOUT_PATH = "/opt/tbotspython/saved_tscope_layout"
LAYOUT_FILE_EXTENSION = "tscopelayout"
LAST_OPENED_LAYOUT_PATH = (
    f"{SAVED_LAYOUT_PATH}/last_opened_tscope_layout.{LAYOUT_FILE_EXTENSION}"
)

THUNDERSCOPE_HELP_TEXT = textwrap.dedent(
    f"""
    <h3>General Controls</h3><br>
    
    <b><code>I:</code></b> Identify robots, toggle robot ID visibility<br>
    <b><code>M:</code></b> Toggle measure mode<br>
    <b><code>S:</code></b> Toggle visibility of robot/ball speed visualization<br>
    <b><code>Ctrl + Space:</code></b> Stop AI vs AI simulation<br>
    <b><code>Number Keys:</code></b> Position camera to preset view<br>
    <b><code>Shift + Left Click:</code></b> Place the ball at the cursor<br>
    <b><code>Shift + Left Click Drag:</code></b> Place the ball at the cursor and kick it<br>

    <h3>Camera Controls</h3><br>

    <b>Orbit:</b> Left click and drag mouse<br>
    <b>Pan:</b> Hold Ctrl while dragging OR drag with middle mouse button<br>
    <b>Zoom:</b> Scrollwheel<br>

    <h3>Measure Mode</h3><br>

    <b><code>M:</code></b> Toggle measure mode / show coordinates<br>
    <b>Shift + Left Click:</b> Place a point<br>
    Placing 2 points will create a distance measurement.<br> 
    Placing 3 points will create an angle measurement.<br> 
    All measurements will be cleared when measure mode is toggled off.

    <h3>Layout Controls</h3><br>

    <b>Pop widget out as window:</b> Double click the widgets' blue bar<br>
    <b>Rearrange/dock widgets:</b> Drag the widgets' blue bar<br><br>
    <b><code>Ctrl + S:</code></b> Save layout<br>
    <b><code>Ctrl + O:</code></b> Open layout<br>
    <b><code>Ctrl + R:</code></b> Remove the current layout file and reset the layout<br><br>
    Layout file (on save) is located at {SAVED_LAYOUT_PATH}<br>

    """
)


def is_field_message_empty(field: Field) -> bool:
    """
    Checks if a field message is empty
    All values in a field message are required so the message will never be None
    So we have to check if the field itself has 0 length
    :param field: the field to check
    :return: True if field message is empty, False if not
    """
    return field.field_x_length == 0


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

    DEFAULT_GRAPHICS_COLOR = QtGui.QColor(255, 255, 255, 128)
    FIELD_LINE_COLOR = QtGui.QColor(255, 255, 255, 200)
    FIELD_LINE_LIGHTER_COLOR = QtGui.QColor(255, 255, 255, 100)
    GOAL_COLOR = QtGui.QColor(200, 200, 200, 255)
    BALL_COLOR = QtGui.QColor(255, 100, 0, 255)
    SIM_BALL_COLOR = QtGui.QColor(255, 100, 0, 150)
    YELLOW_ROBOT_COLOR = QtGui.QColor(255, 255, 0, 255)
    BLUE_ROBOT_COLOR = QtGui.QColor(0, 75, 255, 255)
    TRANSPARENT = QtGui.QColor(0, 0, 0, 0)
    SPEED_VECTOR_COLOR = QtGui.QColor(255, 0, 255, 100)

    DESIRED_ROBOT_LOCATION_OUTLINE = QtGui.QColor(255, 0, 0, 150)
    NAVIGATOR_PATH_COLOR = QtGui.QColor(0, 255, 0, 255)
    NAVIGATOR_OBSTACLE_COLOR = QtGui.QColor(255, 165, 0, 255)
    PASS_VISUALIZATION_COLOR = QtGui.QColor(255, 0, 0, 80)
    BREAKBEAM_TRIPPED_COLOR = QtGui.QColor(255, 0, 0, 255)

    VALIDATION_PASSED_COLOR = QtGui.QColor(0, 200, 0, 255)
    VALIDATION_FAILED_COLOR = QtGui.QColor(200, 0, 0, 255)

    PRIMARY_TEXT_COLOR = QtGui.QColor(255, 255, 255, 255)
    SECONDARY_TEXT_COLOR = QtGui.QColor(255, 255, 255, 160)
    BLACK_TEXT_COLOR = QtGui.QColor(0, 0, 0, 255)

    # Colors for vision pattern
    ROBOT_MIDDLE_BLUE = QtGui.QColor(0, 0, 255, 255)
    PINK = QtGui.QColor(255, 0, 255)
    GREEN = QtGui.QColor(0, 255, 0)

    # Creates a default vision pattern lookup with the actual colors used on the robots
    VISION_PATTERN_LOOKUP = create_vision_pattern_lookup(PINK, GREEN)

    # Colors for black and white vision pattern
    BW_ROBOT_MIDDLE_BLUE = QtGui.QColor(*rgb_to_bw(0, 0, 255))
    BW_PINK = QtGui.QColor(*rgb_to_bw(255, 0, 255))
    BW_GREEN = QtGui.QColor(*rgb_to_bw(0, 255, 0))

    # Creates a black and white vision pattern to indicate a disconnected robot
    BW_VISION_PATTERN_LOOKUP = create_vision_pattern_lookup(BW_PINK, BW_GREEN)


class DepthValues:
    """Constants for depth values controlling the order in which
    graphics are drawn in the 3D visualizer.

    Graphics with greater depth values are drawn later.
    Graphics with negative depth values are drawn before their parent.
    """

    BACKGROUND_DEPTH = -1
    FOREGROUND_DEPTH = 0

    SECONDARY_TEXT_DEPTH = 1
    PRIMARY_TEXT_DEPTH = 2
