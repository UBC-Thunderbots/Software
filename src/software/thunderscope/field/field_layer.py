import math
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from proto.geometry_pb2 import Point, Angle, Segment
from proto.primitive_pb2 import Obstacles
from software.py_constants import *


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
            "brush": pg.mkBrush("w"),
            "tip": "x: {x:.3g}\ny: {y:.3g}\ndata={data}".format,
        }

    def boundingRect(self):
        """boundingRect _must_ indicate the entire area that will be drawn on or
        else we will get artifacts and possibly crashing.
    
        :return: Rectangle that covers the entire field
    
        """
        # TODO (#2398) this rectangle makes no sense, it should be
        # top left x, top left y, width, height. But for some reason
        # that doesn't play nicely with the coordinate system.
        #
        # Instead it is bottom left x, bottom left y, width height.
        return QtCore.QRectF(-9000, -6000, 18000, 12000)

    def createCircle(self, origin: Point, radius):
        """Creates a Rectangle that bounds the circle

        :param origin: Proto Point representing the origin of the circle
        :param radius: The radius of the circle in meters
        :returns: bounding rectangle

        """
        # TODO (#2398) fix this to be top left coordinates, width, height
        x_mm = int(MILLIMETERS_PER_METER * origin.x_meters)
        y_mm = int(MILLIMETERS_PER_METER * origin.y_meters)
        radius_mm = int(MILLIMETERS_PER_METER * radius)
        return QtCore.QRectF(
            int(x_mm - radius_mm),
            int(y_mm - radius_mm),
            int(radius_mm * 2),
            int(radius_mm * 2),
        )

    def drawRobot(self, position: Point, orientation: Angle, painter):
        """
        Draw a robot at the given position and orientation
        :param position: Proto Point representing the position of robot
        :param orientation: Proto Angle representing the orientation of robot
        :param painter: The painter object to draw robot with
        """
        convert_degree = -16
        painter.drawChord(
            self.createCircle(position, ROBOT_MAX_RADIUS_METERS),
            int((math.degrees(orientation.radians) + 45)) * convert_degree,
            270 * convert_degree,
        )

    def drawSegment(self, segment: Segment, painter):
        """
        Draw a segment
        :param segment: Proto Segment representing the segment to draw 
        :param painter: The painter object to draw robot with
        """
        painter.drawLine(
            QtCore.QLine(
                int(segment.start.x_meters * MILLIMETERS_PER_METER),
                int(segment.start.y_meters * MILLIMETERS_PER_METER),
                int(segment.end.x_meters * MILLIMETERS_PER_METER),
                int(segment.end.y_meters * MILLIMETERS_PER_METER),
            )
        )

    def drawObstacles(self, obstacles: Obstacles, painter):
        """
        Draw a segment
        :param obstacles: Proto Obstacle representing a list of polygon and circle obstacles
        :param painter: The painter object to draw robot with
        """
        for poly_obstacle in obstacles.polygon:
            polygon_points = [
                QtCore.QPoint(
                    int(MILLIMETERS_PER_METER * point.x_meters),
                    int(MILLIMETERS_PER_METER * point.y_meters),
                )
                for point in poly_obstacle.points
            ]

            poly = QtGui.QPolygon(polygon_points)
            painter.drawPolygon(poly)

        for circle_obstacle in obstacles.circle:
            painter.drawEllipse(
                self.createCircle(
                    circle_obstacle.origin, circle_obstacle.radius
                )
            )
