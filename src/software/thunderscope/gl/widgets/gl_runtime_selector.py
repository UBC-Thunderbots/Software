from pyqtgraph.Qt.QtWidgets import (
    QDialog,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QComboBox,
)

from software.thunderscope.binary_context_managers.runtime_manager import (
    runtime_manager_instance,
)


class GLRuntimeSelectorDialog(QDialog):
    """Modal that displays the selectable list of runtimes for blue and yellow teams"""

    def __init__(self, parent: QWidget):
        """Initializes the runtime selector modal, displaying the same list of installed
        runtimes for both the blue and yellow teams.

        :param parent: the modal's parent
        """
        super().__init__(parent)

        runtime_options = runtime_manager_instance.fetch_installed_runtimes()
        runtime_config = runtime_manager_instance.fetch_runtime_config()

        # Put selected runtimes from config at start of list
        blue_runtimes = [runtime_config.blue_runtime] + [
            x for x in runtime_options if x != runtime_config.blue_runtime
        ]
        yellow_runtimes = [runtime_config.yellow_runtime] + [
            x for x in runtime_options if x != runtime_config.yellow_runtime
        ]

        self.setWindowTitle("Select Runtimes")
        self.setModal(True)
        self.setMinimumWidth(400)

        layout = QVBoxLayout(self)

        # Blue runtime
        layout.addWidget(QLabel("<b>Blue Runtime</b>"))
        self.blue_menu = QComboBox()
        self.blue_menu.addItems(blue_runtimes)
        self.blue_menu.currentTextChanged.connect(self._on_blue_changed)
        self._blue_selection = self.blue_menu.currentText()
        layout.addWidget(self.blue_menu)

        layout.addSpacing(10)

        # Yellow runtime
        layout.addWidget(QLabel("<b>Yellow Runtime</b>"))
        self.yellow_menu = QComboBox()
        self.yellow_menu.addItems(yellow_runtimes)
        self.yellow_menu.currentTextChanged.connect(self._on_yellow_changed)
        self._yellow_selection = self.yellow_menu.currentText()
        layout.addWidget(self.yellow_menu)

        # Restart note
        layout.addSpacing(15)
        restart_note = QLabel(
            "<i>Note: Restart Thunderscope for changes to take effect.</i>"
        )
        layout.addWidget(restart_note)

        # Done button
        layout.addSpacing(15)
        button_row = QHBoxLayout()
        button_row.addStretch()

        done_button = QPushButton("Done")
        done_button.clicked.connect(self._on_done)
        button_row.addWidget(done_button)

        layout.addLayout(button_row)

    def _on_blue_changed(self, value: str) -> None:
        """Stores currently selected runtime for blue team

        :param value: the value of the selected option
        """
        self._blue_selection = value

    def _on_yellow_changed(self, value: str) -> None:
        """Stores currently selected runtime for yellow team

        :param value: the value of the selected option
        """
        self._yellow_selection = value

    def _on_done(self) -> None:
        """Commits the selected runtimes and closes the modal."""
        runtime_manager_instance.load_selected_runtimes(
            self._yellow_selection, self._blue_selection
        )

        self.close()
