from pyqtgraph.Qt import QtGui
import os


def get_icon(path: os.PathLike, color: str = None) -> QtGui.QPixmap:
    """
    Returns a QPixmap of the icon from the given path, with the given color if defined
    :param path: the path of the icon file
    :param color: the color the icon should be
    :return: a QPixmap of the icon with the right color
    """
    img = QtGui.QPixmap(path)
    if color:
        qp = QtGui.QPainter(img)
        qp.setCompositionMode(QtGui.QPainter.CompositionMode.CompositionMode_SourceIn)
        qp.fillRect(img.rect(), QtGui.QColor(color))
        qp.end()
    return QtGui.QIcon(img)
