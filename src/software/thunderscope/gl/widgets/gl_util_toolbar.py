from typing import override
from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
from software.thunderscope.gl.widgets.toolbars.runtime.gl_runtime_toolbar import (
    GLRuntimeToolbar,
)
from software.thunderscope.proto_unix_io import ProtoUnixIO
from pyqtgraph.Qt.QtWidgets import *

from software.thunderscope.gl.widgets.toolbars.gl_gamecontroller_toolbar import (
    GLGamecontrollerToolbar,
)
from software.thunderscope.gl.widgets.toolbars.gl_sandbox_state_toolbar import (
    GLSandboxStateToolbar,
)


class GLUtilToolbar(GLToolbar):
    """A toolbar with utility controls for GameController and Installing Runtimes"""

    GAME_CONTROLLER_URL = "http://localhost:8081"

    def __init__(
        self,
        parent: QWidget,
        proto_unix_io: ProtoUnixIO,
        friendly_color_yellow: bool,
        sandbox_mode: bool = False,
    ):
        """Initializes the toolbar and constructs its layout

        :param parent: the parent to overlay this toolbar over
        :param proto_unix_io the ProtoUnixIO object to send the manual gamecontroller commands to
        :param friendly_color_yellow True if yellow is friendly team, False if not
        """
        super(GLUtilToolbar, self).__init__(parent=parent)

        self.sandbox_mode = sandbox_mode

        self.gc_toolbar = GLGamecontrollerToolbar(
            toolbar=self,
            proto_unix_io=proto_unix_io,
            friendly_color_yellow=friendly_color_yellow,
        )

        self.layout().addStretch()
        self.add_separator()

        self.runtime_toolbar = GLRuntimeToolbar(toolbar=self)

        if self.sandbox_mode:
            self.add_separator()
            self.sandbox_state_toolbar = GLSandboxStateToolbar(
                toolbar=self, proto_unix_io=proto_unix_io
            )

    @override
    def refresh(self) -> None:
        """Refreshes the UI to update toolbar position"""
        self.move(0, self.parentWidget().geometry().bottom() - self.height())

        if self.sandbox_mode:
            self.sandbox_state_toolbar.refresh()
