from pyqtgraph.Qt import QtWidgets
from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
import qtawesome as qta
from typing import override


class GLSimulatedTestToolbar(GLToolbar):
    """A toolbar with controls to run simulated tests within Thunderscope"""

    def __init__(self, parent: QtWidgets.QWidget):
        """Initializes the toolbar and constructs its layout

        :param parent: the parent to overlay this toolbar over
        """
        super(GLSimulatedTestToolbar, self).__init__(parent=parent)

        self.run_test_button = self.setup_icon_button(
            qta.icon("fa5s.play"),
            "Runs simluated test",
            self.__run_test,
        )

        self.layout().addWidget(QtWidgets.QLabel("<b>Simulated Tests</b>"))
        self.add_separator(self.layout())
        self.layout().addWidget(self.run_test_button)

    @override
    def refresh(self) -> None:
        """Refreshes the UI to update toolbar position"""
        self.move(0, self.parentWidget().geometry().bottom() - self.height())

    def __run_test(self):
        print("RUN TEST")
