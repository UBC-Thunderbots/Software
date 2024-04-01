from pyqtgraph.Qt import QtCore, QtGui
from proto.import_all_protos import *
from enum import Enum, IntEnum
from proto.robot_log_msg_pb2 import LogLevel

import textwrap

SIM_TICK_RATE_MS = 16


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

    def __str__(self) -> str:
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
    PHYSICAL_ESTOP: A physical estop is needed to run thunderscope, throws an exception if none is plugged in
    """

    DISABLE_ESTOP = 0
    KEYBOARD_ESTOP = 1
    PHYSICAL_ESTOP = 2


# the maximum packet / world loss percent indicated by UI
MAX_ACCEPTABLE_PACKET_LOSS_PERCENT = 30


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

# How long AI vs AI runs before ending in CI
CI_DURATION_S = 180

MULTI_PLANE_POINTS = 3

ROBOT_RADIUS = 25

BALL_HEIGHT_EFFECT_MULTIPLIER = 3

# Time (in seconds) to sleep / delay the loop iteration for different protos
# in robot communications
ROBOT_COMMUNICATIONS_TIMEOUT_S = 0.02

# time between each refresh of thunderscope in milliseconds
THUNDERSCOPE_REFRESH_INTERVAL_MS = 10

ROBOT_FATAL_TIMEOUT_S = 5
# Max time (in seconds) tolerated between repeated crash protos until
# crash alert occurs
ROBOT_CRASH_TIMEOUT_S = 5

# FOV in degrees for top-down orthographic view
ORTHOGRAPHIC_FOV_DEGREES = 1.0

# LogLevel to string conversion map
LOG_LEVEL_STR_MAP = {
    LogLevel.DEBUG: "DEBUG",
    LogLevel.INFO: "INFO",
    LogLevel.WARNING: "WARNING",
    LogLevel.FATAL: "FATAL",
    LogLevel.CONTRACT: "CONTRACT",
}

# Paths to check for estop when running diagnostics
ESTOP_PATH_1 = "/dev/ttyACM0"
ESTOP_PATH_2 = "/dev/ttyUSB0"

# Mapping between RobotStatus Error Codes and their dialog messages
ERROR_CODE_MESSAGES = {
    ErrorCode.HIGH_CAP: "High Cap",
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
    <b><code>Ctrl + Shift + Left Double Click:</code></b>
    <ul style="margin: 0;">
    <li>If no robot is present at cursor, adds a new friendly robot there</li>
    <li>If a friendly robot is present at cursor, removes it</li>
    </ul>
    <b><code>Ctrl + Shift + Left Click Drag:</code></b> Moves a friendly robot along with the cursor

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


def create_vision_pattern_lookup(color1: QtGui.QColor, color2: QtGui.QColor) -> dict:
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


def rgb_to_bw(r: int, g: int, b: int) -> tuple:
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

    DESIRED_ROBOT_LOCATION_OUTLINE = QtGui.QColor(255, 0, 0, 255)
    NAVIGATOR_PATH_COLOR = QtGui.QColor(0, 255, 0, 255)
    NAVIGATOR_OBSTACLE_COLOR = QtGui.QColor(255, 80, 0, 100)
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

    BENEATH_BACKGROUND_DEPTH = -2
    BACKGROUND_DEPTH = -1
    FOREGROUND_DEPTH = 0
    ABOVE_FOREGROUND_DEPTH = 1
    OVERLAY_DEPTH = 2


class TrailValues:
    """Constants for Trails Visualization Layer in Thunderscope."""

    DEFAULT_TRAIL_LENGTH = 20
    DEFAULT_TRAIL_SAMPLING_RATE = 0


# TODO: come up with less ambiguous names
class InputEventType(Enum):
    MOVE_X = 1
    MOVE_Y = 2
    ROTATE = 3
    KICK = 4
    CHIP = 5
    KICK_POWER = 6
    DRIBBLER_SPEED = 7
    DRIBBLER_ENABLE_1 = 8
    DRIBBLER_ENABLE_2 = 9


class ControllerConstants:
    XboxEventCodes = {
        # Name: "ABS_X", Type: EV_ABS
        # AbsInfo: value=1242, min=-32768, max=32767, fuzz=16, flat=128, resolution=0
        # Canonical: Left joystick X-axis
        InputEventType.MOVE_X: 0,

        # Name: "ABS_Y", Type: EV_ABS
        # AbsInfo: value=425, min=-32768, max=32767, fuzz=16, flat=128, resolution=0
        # Canonical: Left joystick Y-axis
        InputEventType.MOVE_Y: 1,

        # Name: "ABS_RX", Type: EV_ABS
        # AbsInfo: value=-418, min=-32768, max=32767, fuzz=16, flat=128, resolution=0
        # Canonical: Right joystick X-axis
        InputEventType.ROTATE: 3,

        # Name: "BTN_A", Type: EV_KEY
        # Canonical: "A" Button
        InputEventType.KICK: 304,

        # Name: "BTN_Y", Type: EV_KEY
        # Canonical: "Y" Button
        InputEventType.CHIP: 308,

        # Name: "ABS_HAT0X", Type: EV_ABS
        # AbsInfo: value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0
        # Canonical: D-pad X-axis
        InputEventType.KICK_POWER: 16,

        # Name: "ABS_HAT0Y", Type: EV_ABS
        # AbsInfo: value=0, min=-1, max=1, fuzz=0, flat=0, resolution=0
        # Canonical: D-pad Y-axis
        InputEventType.DRIBBLER_SPEED: 17,

        # Name: "ABS_Z", Type: EV_ABS
        # AbsInfo: value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0
        # Canonical: Left trigger
        InputEventType.DRIBBLER_ENABLE_1: 2,

        # Name: "ABS_RZ", Type: EV_ABS
        # AbsInfo: value=0, min=0, max=1023, fuzz=0, flat=0, resolution=0
        # Canonical: Right trigger
        InputEventType.DRIBBLER_ENABLE_2: 5
    }

    CONTROLLER_NAME_CODES_MAP = {
        "Microsoft Xbox One X pad": XboxEventCodes,
        "Microsoft X-Box 360 pad": XboxEventCodes,
    }

    INPUT_DELAY_THRESHOLD = 0.005

    XBOX_MAX_RANGE = 32768.0
    XBOX_BUTTON_MAX_RANGE = 1024.0
    DEADZONE_PERCENTAGE = 0.30

    MAX_LINEAR_SPEED_METER_PER_S = 2.0
    MAX_ANGULAR_SPEED_RAD_PER_S = 20.0

    DRIBBLER_STEPPER = 100.0
    DRIBBLER_INDEFINITE_SPEED = -10000.0

    POWER_STEPPER = 100.0
    MIN_POWER = 1000.0
    MAX_POWER = 20000.0
