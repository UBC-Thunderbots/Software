from pyqtgraph.Qt import QtCore, QtWidgets


class RerunTestWidget(QtWidgets.QWidget):
    """A button to rerun the currently running field test.

    The test runs in a background thread, so the runner only sets test_running
    and the button state is updated from it on refresh, on the Qt thread.
    """

    rerun_requested = QtCore.pyqtSignal()

    def __init__(self) -> None:
        """Initialize the RerunTestWidget"""
        super(RerunTestWidget, self).__init__()

        self.test_running = False

        self.rerun_button = QtWidgets.QPushButton("Rerun Test")
        self.rerun_button.clicked.connect(self.rerun_requested.emit)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.rerun_button)
        self.setLayout(layout)

    def refresh(self) -> None:
        """Only allow a rerun once the current run has finished"""
        self.rerun_button.setEnabled(not self.test_running)
        self.rerun_button.setText(
            "Test Running..." if self.test_running else "Rerun Test"
        )
