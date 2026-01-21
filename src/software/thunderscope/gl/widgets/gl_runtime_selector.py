from pyqtgraph.Qt.QtWidgets import (
    QDialog,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QComboBox,
)
from typing import Callable


class GLRuntimeSelectorDialog(QDialog):
    """Modal that displays the selectable list of runtimes for yellow and blue teams"""

    def __init__(
        self,
        parent: QWidget,
        runtime_options: list[str],
        on_runtimes_selected: Callable[[str, str], None],
    ):
        """Initializes the runtime selector modal, displaying the same list of installed
        runtimes for both the blue and yellow teams.

        :param parent: the modal's parent
        :param runtime_options: the list of runtime options to display in both menus
        :param on_runtimes_selected: the callback for runtime selection
        """

        super().__init__(parent)

        self.on_runtimes_selected = on_runtimes_selected

        self.setWindowTitle("Select Runtimes")
        self.setModal(True)
        self.setMinimumWidth(400)

        layout = QVBoxLayout(self)

        # Yellow runtime
        layout.addWidget(QLabel("<b>Yellow Runtime</b>"))
        self.yellow_menu = QComboBox()
        self.yellow_menu.addItems(runtime_options)
        self.yellow_menu.currentTextChanged.connect(self._on_yellow_changed)
        self._yellow_selection = self.yellow_menu.currentText()
        layout.addWidget(self.yellow_menu)

        # Blue runtime
        layout.addSpacing(10)
        layout.addWidget(QLabel("<b>Blue Runtime</b>"))
        self.blue_menu = QComboBox()
        self.blue_menu.addItems(runtime_options)
        self.blue_menu.currentTextChanged.connect(self._on_blue_changed)
        self._blue_selection = self.blue_menu.currentText()
        layout.addWidget(self.blue_menu)

        # Restart note
        layout.addSpacing(15)
        restart_note = QLabel("<i>Note: Restart Thunderscope for changes to take effect.</i>")
        layout.addWidget(restart_note)

        # Done button
        layout.addSpacing(15)
        button_row = QHBoxLayout()
        button_row.addStretch()

        done_button = QPushButton("Done")
        done_button.clicked.connect(self._on_done)
        button_row.addWidget(done_button)

        layout.addLayout(button_row)

    def _on_yellow_changed(self, value: str) -> None:
        """Stores currently selected runtime for yellow team

        :param value: the value of the selected option
        """
        self._yellow_selection = value

    def _on_blue_changed(self, value: str) -> None:
        """Stores currently selected runtime for blue team

        :param value: the value of the selected option
        """
        self._blue_selection = value

    def _on_done(self) -> None:
        """Commits the selected runtimes and closes the modal."""
        self.on_runtimes_selected(self._yellow_selection, self._blue_selection)

        self.close()