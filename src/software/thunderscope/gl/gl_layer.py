import pyqtgraph as pg

class GLLayer(pg.GraphicsObject):

    def __init__(self):
        pg.GraphicsObject.__init__(self)

        # Options for the layer, used to configure the legend
        self.opts = {
            "pxMode": True,
            "useCache": True,
            "antialias": True,
            "name": None,
            "symbol": "o",
            "size": 7,
            "pen": pg.mkPen("w"),
            "brush": pg.mkBrush("w"),
            "tip": "x: {x:.3g}\ny: {y:.3g}\ndata={data}".format,
        }

    def updateGraphics(self):
        """Updates the GLGraphicsItems in this layer

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - list of the added GLGraphicsItems
            - removed_graphics - list of the removed GLGraphicsItems
        
        """
        raise NotImplementedError("Abstract class method called: updateGraphics")

            