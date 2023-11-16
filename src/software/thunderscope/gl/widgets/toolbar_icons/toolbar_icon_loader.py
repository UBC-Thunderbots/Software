from pyqtgraph.Qt import QtGui
import os


class GLFieldToolbarIconLoader:
    """
    Stores icons for the GL Field Toolbar widget

    Since they are class level variables, they are initialized only once
    when they are first accessed
    Which saves the cost of loading them every time
    """

    UNDO_ICON = None
    REDO_ICON = None
    PAUSE_ICON = None
    HELP_ICON = None
    RESET_ICON = None
    VIEW_ICON = None
    MEASURE_ICON = None


def get_icon(path: os.PathLike, color: str) -> QtGui.QPixmap:
    """
    Returns a QPixmap of the icon from the given path, with the given color
    :param path: the path of the icon file
    :param color: the color the icon should be
    :return: a QPixmap of the icon with the right color
    """
    img = QtGui.QPixmap(path)
    qp = QtGui.QPainter(img)
    qp.setCompositionMode(QtGui.QPainter.CompositionMode.CompositionMode_SourceIn)
    qp.fillRect(img.rect(), QtGui.QColor(color))
    qp.end()
    return QtGui.QIcon(img)


def get_undo_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Undo icon pixmap as a GLFieldToolbarIconLoader attribute

    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLFieldToolbarIconLoader.UNDO_ICON:
        GLFieldToolbarIconLoader.UNDO_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/undo.svg", color
        )

    return GLFieldToolbarIconLoader.UNDO_ICON


def get_redo_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Redo icon pixmap as a GLFieldToolbarIconLoader attribute

    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLFieldToolbarIconLoader.REDO_ICON:
        GLFieldToolbarIconLoader.REDO_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/redo.svg", color
        )

    return GLFieldToolbarIconLoader.REDO_ICON


def get_pause_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Pause icon pixmap as a GLFieldToolbarIconLoader attribute

    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLFieldToolbarIconLoader.PAUSE_ICON:
        GLFieldToolbarIconLoader.PAUSE_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/pause.svg", color
        )

    return GLFieldToolbarIconLoader.PAUSE_ICON


def get_help_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Help icon pixmap as a GLFieldToolbarIconLoader attribute

    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLFieldToolbarIconLoader.HELP_ICON:
        GLFieldToolbarIconLoader.HELP_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/help.svg", color
        )

    return GLFieldToolbarIconLoader.HELP_ICON


def get_reset_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Reset icon pixmap as a GLFieldToolbarIconLoader attribute

    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLFieldToolbarIconLoader.RESET_ICON:
        GLFieldToolbarIconLoader.RESET_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/reset.svg", color
        )

    return GLFieldToolbarIconLoader.RESET_ICON


def get_view_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the View icon pixmap as a GLFieldToolbarIconLoader attribute

    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLFieldToolbarIconLoader.VIEW_ICON:
        GLFieldToolbarIconLoader.VIEW_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/view.svg", color
        )

    return GLFieldToolbarIconLoader.VIEW_ICON


def get_measure_icon(color: str) -> QtGui.QPixmap:
    """
    Loads the Measure icon pixmap as a GLFieldToolbarIconLoader attribute

    :param color: the color the icon should be initialized with if not already created
    :return: the icon pixmap
    """
    if not GLFieldToolbarIconLoader.MEASURE_ICON:
        GLFieldToolbarIconLoader.MEASURE_ICON = get_icon(
            "software/thunderscope/gl/widgets/toolbar_icons/measure.svg", color
        )

    return GLFieldToolbarIconLoader.MEASURE_ICON
