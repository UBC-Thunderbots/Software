from pyqtgraph.opengl import *
from pyqtgraph.opengl import *

import numpy as np

class GLHeatmap(GLMeshItem):
    """
    Displays a surface plot on a regular x,y grid
    """

    def __init__(self, x=None, y=None, colors=None, parentItem=None, **kwds):
        """
        The x, y, z, and colors arguments are passed to setData().
        All other keyword arguments are passed to GLMeshItem.__init__().
        """
        self.__meshdata = MeshData()
        super().__init__(
            parentItem=parentItem, 
            meshdata=self.__meshdata, 
            smooth=False, 
            computeNormals=False,
            **kwds
        )

    def setData(self, x, y, colors):
        ## Generate vertices
        xx, yy = np.meshgrid(x, y)
        vertices = np.dstack((xx, yy)).reshape(-1, 2)
        vertices = np.hstack((vertices, np.zeros((vertices.shape[0], 1), dtype=np.int64)))

        # Generate faces
        rows, cols = len(y) - 1, len(x) - 1
        faces = np.empty((cols*rows*2, 3), dtype=np.uint)
        rowtemplate1 = np.arange(cols).reshape(cols, 1) + np.array([[0, 1, cols+1]])
        rowtemplate2 = np.arange(cols).reshape(cols, 1) + np.array([[cols+1, 1, cols+2]])
        for row in range(rows):
            start = row * cols * 2 
            faces[start:start+cols] = rowtemplate1 + row * (cols+1)
            faces[start+cols:start+(cols*2)] = rowtemplate2 + row * (cols+1)

        colors = np.repeat(colors, repeats=2, axis=0).reshape(len(faces), 4)

        ## Update MeshData
        self.__meshdata.setVertexes(vertices)
        self.__meshdata.setFaces(faces)
        self.__meshdata.setFaceColors(colors)
        self.meshDataChanged()

    def clear(self):
        self.__meshdata = MeshData()
        self.setMeshData(meshdata=self.__meshdata)