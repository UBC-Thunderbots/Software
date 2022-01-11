import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui


class FieldLayer(pg.GraphicsObject):
    def __init__(self):
        pg.GraphicsObject.__init__(self)

        # options for the layer, used to configure the legend
        self.opts = {
            "pxMode": True,
            "useCache": True,
            "antialias": True,
            "name": None,
            "symbol": "o",
            "size": 7,
            "pen": pg.mkPen("w"),
            "brush": pg.mkBrush("r"),
            "tip": "x: {x:.3g}\ny: {y:.3g}\ndata={data}".format,
        }

    def boundingRect(self):
        """boundingRect _must_ indicate the entire area that will be drawn on or
        else we will get artifacts and possibly crashing.
    
        :return: Rectangle that covers the entire field
    
        """
        # TODO this makes no sense
        return QtCore.QRectF(-9000, -6000, 18000, 12000)

    def createCircle(self, x, y, radius):
        """Creates a Rectangle that bounds the circle

        :param x: The x position
        :param y: The y position
        :param radius: The radius of the circle
        :returns: bounding rectangle

        """
        return QtCore.QRectF(
            int(x - radius), int(y - radius), int(radius * 2), int(radius * 2)
        )
