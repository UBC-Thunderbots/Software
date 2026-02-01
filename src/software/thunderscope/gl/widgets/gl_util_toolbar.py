from typing import override
from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
from software.thunderscope.gl.widgets.toolbars.generate_test.gl_generate_test_toolbar import (
    GLGenerateTestToolbar,
)
from software.thunderscope.gl.widgets.toolbars.runtime.gl_runtime_toolbar import (
    GLRuntimeToolbar,
)
from software.thunderscope.proto_unix_io import ProtoUnixIO
from pyqtgraph.Qt.QtWidgets import *

from software.thunderscope.gl.widgets.toolbars.gl_gamecontroller_toolbar import (
    GLGamecontrollerToolbar,
)


class GLUtilToolbar(GLToolbar):
    """A toolbar with utility controls for GameController and Installing Runtimes"""

    GAME_CONTROLLER_URL = "http://localhost:8081"

    def __init__(
        self,
        parent: QWidget,
        proto_unix_io: ProtoUnixIO,
        friendly_color_yellow: bool,
        generate_tests: bool = False,
    ):
        """Initializes the toolbar and constructs its layout

        :param parent: the parent to overlay this toolbar over
        :param proto_unix_io the ProtoUnixIO object to send the manual gamecontroller commands to
        :param friendly_color_yellow True if yellow is friendly team, False if not
        :param generate_tests True if the test generate utils should be displayed
        """
        super(GLUtilToolbar, self).__init__(parent=parent)

        self.generate_tests = generate_tests

        self.gc_toolbar = GLGamecontrollerToolbar(
            toolbar=self,
            proto_unix_io=proto_unix_io,
            friendly_color_yellow=friendly_color_yellow,
        )

        self.layout().addStretch()
        self.add_separator()

        self.runtime_toolbar = GLRuntimeToolbar(toolbar=self)

        if self.generate_tests:
            self.add_separator()

            self.test_gen_toolbar = GLGenerateTestToolbar(toolbar=self)

    @override
    def refresh(self) -> None:
        """Refreshes the UI to update toolbar position"""
        self.move(0, self.parentWidget().geometry().bottom() - self.height())

        if self.generate_tests:
            self.test_gen_toolbar.refresh()
