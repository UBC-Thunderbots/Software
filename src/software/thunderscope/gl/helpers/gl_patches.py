from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem
from pyqtgraph.opengl.GLViewWidget import GLViewMixin
from pyqtgraph.opengl.shaders import ShaderProgram
from pyqtgraph.Qt import QtGui
from typing import Callable


def GLGraphicsItem_setParentItem_patched(self, parent: GLGraphicsItem) -> None:
    """Patched version of GLGraphicsItem.setParentItem that properly
    removes this item from the scenegraph when the parent param is None.

    From the original pyqtgraph documentation:
    Sets this item's parent in the scenegraph hierarchy.

    :param parent: the GLGraphicsItem to set as the parent of this item.
                   If None, this item is removed from the scenegraph.
    """
    if self._GLGraphicsItem__parent:
        self._GLGraphicsItem__parent._GLGraphicsItem__children.remove(self)
    self._GLGraphicsItem__parent = None
    if self.view():
        self.view().removeItem(self)

    if parent:
        parent._GLGraphicsItem__children.add(self)
        self._GLGraphicsItem__parent = parent
        if parent.view():
            parent.view().addItem(self)


def GLViewMixin_addItem_patched(self, item: GLGraphicsItem) -> None:
    """Patched version of GLViewMixin.addItem that properly adds all
    the item and all its descendants to the scene.

    :param item: the item to add to the scene
    """
    items_to_add = [item]

    while items_to_add:
        item_to_add = items_to_add.pop()
        self.items.append(item_to_add)
        item_to_add._setView(self)

        if self.isValid():
            item_to_add.initialize()

        for child in item_to_add.childItems():
            items_to_add.append(child)

    self.update()


def GLViewMixin_removeItem_patched(self, item: GLGraphicsItem) -> None:
    """Patched version of GLViewMixin.removeItem that properly removes
    the given item and its descendants from the scene.

    :param item: the item to remove from the scene
    """
    items_to_remove = [item]

    while items_to_remove:
        item_to_remove = items_to_remove.pop()
        self.items.remove(item_to_remove)
        item_to_remove._setView(None)

        for child in item_to_remove.childItems():
            items_to_remove.append(child)

    self.update()


def ShaderProgram_program_patched(original: Callable) -> Callable:
    """Returns a patched version of ShaderProgram.program that forces
    recompilation of the shader program when the OpenGL context changes.

    :param original: the original ShaderProgram.program method
    :return: the patched ShaderProgram.program method
    """

    def patched(self):
        ctx = QtGui.QOpenGLContext.currentContext()
        if not hasattr(self, "gl_ctx") or self.gl_ctx != ctx:
            self.gl_ctx = ctx
            self.prog = None
            for shader in self.shaders:
                shader.compiled = None
        return original(self)

    return patched


GLGraphicsItem.setParentItem = GLGraphicsItem_setParentItem_patched
GLViewMixin.addItem = GLViewMixin_addItem_patched
GLViewMixin.removeItem = GLViewMixin_removeItem_patched
ShaderProgram.program = ShaderProgram_program_patched(ShaderProgram.program)
