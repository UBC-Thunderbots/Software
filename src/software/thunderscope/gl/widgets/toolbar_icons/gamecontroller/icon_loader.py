from pyqtgraph.Qt import QtGui
from software.thunderscope.gl.widgets.icon_loader import get_icon


class GLGamecontrollerToolbarIconLoader:
    """
    Stores icones for the GL Gamecontroller Toolbar widget

    Since they are class level variables, they are initialized only once
    when they are first accessed
    Which saves the cost of loading them every time
    """

    HALT_ICON = None
    STOP_ICON = None
    FORCE_START_ICON = None
    NORMAL_START_ICON = {}
    YELLOW_ICON = None
    BLUE_ICON = None
    OPEN_WINDOW_ICON = None


def get_halt_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Halt icon pixmap as a GLGamecontrollerToolbarIconLoader attribute
    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLGamecontrollerToolbarIconLoader.HALT_ICON:
        GLGamecontrollerToolbarIconLoader.HALT_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/gamecontroller/halt.svg",
            color,
        )

    return GLGamecontrollerToolbarIconLoader.HALT_ICON


def get_stop_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Stop icon pixmap as a GLGamecontrollerToolbarIconLoader attribute
    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLGamecontrollerToolbarIconLoader.STOP_ICON:
        GLGamecontrollerToolbarIconLoader.STOP_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/gamecontroller/stop.svg",
            color,
        )

    return GLGamecontrollerToolbarIconLoader.STOP_ICON


def get_normal_start_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Normal Start icon pixmap as a GLGamecontrollerToolbarIconLoader attribute
    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if color not in GLGamecontrollerToolbarIconLoader.NORMAL_START_ICON:
        GLGamecontrollerToolbarIconLoader.NORMAL_START_ICON[color] = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/gamecontroller/normal_start.svg",
            color,
        )

    return GLGamecontrollerToolbarIconLoader.NORMAL_START_ICON[color]


def get_force_start_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Force Start icon pixmap as a GLGamecontrollerToolbarIconLoader attribute
    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLGamecontrollerToolbarIconLoader.FORCE_START_ICON:
        GLGamecontrollerToolbarIconLoader.FORCE_START_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/gamecontroller/force_start.svg",
            color,
        )

    return GLGamecontrollerToolbarIconLoader.FORCE_START_ICON


def get_yellow_icon() -> QtGui.QPixmap:
    """
    Loads the Yellow icon pixmap as a GLGamecontrollerToolbarIconLoader attribute
    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLGamecontrollerToolbarIconLoader.YELLOW_ICON:
        GLGamecontrollerToolbarIconLoader.YELLOW_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/gamecontroller/yellow.svg"
        )

    return GLGamecontrollerToolbarIconLoader.YELLOW_ICON


def get_blue_icon() -> QtGui.QPixmap:
    """
    Loads the Blue icon pixmap as a GLGamecontrollerToolbarIconLoader attribute
    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLGamecontrollerToolbarIconLoader.BLUE_ICON:
        GLGamecontrollerToolbarIconLoader.BLUE_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/gamecontroller/blue.svg"
        )

    return GLGamecontrollerToolbarIconLoader.BLUE_ICON


def get_open_window_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Open Window icon pixmap as a GLGamecontrollerToolbarIconLoader attribute
    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLGamecontrollerToolbarIconLoader.OPEN_WINDOW_ICON:
        GLGamecontrollerToolbarIconLoader.OPEN_WINDOW_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/gamecontroller/open_window.svg",
            color,
        )

    return GLGamecontrollerToolbarIconLoader.OPEN_WINDOW_ICON
