from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem
import pyqtgraph as pg

from typing import Optional, Tuple

import numpy as np
import numpy.typing as npt


class GLHeatmap(GLMeshItem):
    """
    Displays a heatmap on the cartesian plane (i.e. x-y plane)
    """

    # Float between 0.0 - 1.0 representing alpha transparency of heatmap
    HEATMAP_ALPHA = 0.8

    def __init__(
        self, parent_item: Optional[GLGraphicsItem] = None, color_map: pg.ColorMap = None,
    ) -> None:
        """Initialize the GLHeatmap
        
        :param parent_item: The parent item of the graphic
        :param color_map: ColorMap to map scalar data values to the color space.
                          If no ColorMap is provided, the default is a grayscale ColorMap.

        """
        self.x_length = 0
        self.y_length = 0
        self.color_map = color_map if color_map else pg.colormap.get("CET-L1")
        self.meshdata = MeshData()

        super().__init__(
            parentItem=parent_item,
            meshdata=self.meshdata,
            smooth=False,
            computeNormals=False,
        )
        self.setGLOptions("additive")

    def set_dimensions(self, x_length: float = 0, y_length: float = 0) -> None:
        """Set the dimensions of the heatmap
        
        :param x_length: The length of the heatmap in the x direction
        :param y_length: The length of the heatmap in the y direction

        """
        if x_length == 0 or y_length == 0:
            return

        if self.x_length == x_length and self.y_length == y_length:
            return

        self.x_length = x_length
        self.y_length = y_length

    def setData(self, data: npt.ArrayLike) -> None:
        """Update the data in this heatmap

        :param data: A 2D matrix representing the data to display in the heatmap

        """
        rows = data.shape[0]
        cols = data.shape[1]

        # Generate x and y coordinate vectors
        x = np.arange(
            start=-(self.x_length / 2),
            stop=(self.x_length / 2) + (self.x_length / cols),
            step=self.x_length / cols,
        )
        y = np.arange(
            start=-(self.y_length / 2),
            stop=(self.y_length / 2) + (self.y_length / rows),
            step=self.y_length / rows,
        )

        # Generate vertices
        xx, yy = np.meshgrid(x, y)
        vertices = np.dstack((xx, yy)).reshape(-1, 2)
        vertices = np.hstack(
            (vertices, np.zeros((vertices.shape[0], 1), dtype=np.uint))
        )

        # Generate faces
        face_rows, face_cols = len(y) - 1, len(x) - 1
        faces = np.empty((face_cols * face_rows * 2, 3), dtype=np.uint)
        rowtemplate1 = np.arange(face_cols).reshape(face_cols, 1) + np.array(
            [[0, 1, face_cols + 1]]
        )
        rowtemplate2 = np.arange(face_cols).reshape(face_cols, 1) + np.array(
            [[face_cols + 1, 1, face_cols + 2]]
        )
        for row in range(face_rows):
            start = row * face_cols * 2
            faces[start : start + face_cols] = rowtemplate1 + row * (face_cols + 1)
            faces[start + face_cols : start + (face_cols * 2)] = rowtemplate2 + row * (
                face_cols + 1
            )

        # Compute face color per data point
        colors = self.color_map.map(data.ravel(), mode=pg.ColorMap.FLOAT)
        colors[:, 3] = GLHeatmap.HEATMAP_ALPHA
        colors = colors.reshape(rows, cols, 4)
        colors = np.repeat(colors, repeats=2, axis=0).reshape(len(faces), 4)

        # Update meshdata
        self.meshdata.setVertexes(vertices)
        self.meshdata.setFaces(faces)
        self.meshdata.setFaceColors(colors)
        self.meshDataChanged()
