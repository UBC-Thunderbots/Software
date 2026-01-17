from pyqtgraph.Qt.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QComboBox,
)
from typing import Callable


class BackendSelectionDialog(QDialog):
    """A modal dialog to select AI backends for the friendly and opponent teams"""

    def __init__(
        self,
        parent,
        backend_options: list[str],
        on_friendly_selected: Callable[[str], None],
        on_opponent_selected: Callable[[str], None],
    ):
        """Initializes the backend selection modal and constructs its layout

        :param parent: the parent widget
        :param backend_options: the list of backend options to display in both menus
        :param on_friendly_selected: the callback for friendly backend selection
        :param on_opponent_selected: the callback for opponent backend selection
        """

        super().__init__(parent)

        self.on_friendly_selected = on_friendly_selected
        self.on_opponent_selected = on_opponent_selected

        self.setWindowTitle("Select Backends")
        self.setModal(True)
        self.setMinimumWidth(400)

        layout = QVBoxLayout(self)

        # Friendly backend
        layout.addWidget(QLabel("<b>Friendly Backend</b>"))
        self.friendly_menu = QComboBox()
        self.friendly_menu.addItems(backend_options)
        self.friendly_menu.currentTextChanged.connect(self._on_friendly_changed)
        self._friendly_selection = self.friendly_menu.currentText()
        layout.addWidget(self.friendly_menu)

        # Opponent backend
        layout.addSpacing(10)
        layout.addWidget(QLabel("<b>Opponent Backend</b>"))
        self.opponent_menu = QComboBox()
        self.opponent_menu.addItems(backend_options)
        self.opponent_menu.currentTextChanged.connect(self._on_opponent_changed)
        self._opponent_selection = self.opponent_menu.currentText()
        layout.addWidget(self.opponent_menu)

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

    def _on_friendly_changed(self, value: str) -> None:
        """Stores currently selected backend option for friendly team

        :param value: the value of the selected option
        """
        self._friendly_selection = value

    def _on_opponent_changed(self, value: str) -> None:
        """Stores currently selected backend option for opponent team

        :param value: the value of the selected option
        """
        self._opponent_selection = value

    def _on_done(self) -> None:
        """Commits the selected backend values and closes the dialog.
        Both callbacks are invoked, using the latest selected values.
        """
        self.on_friendly_selected(self._friendly_selection)
        self.on_opponent_selected(self._opponent_selection)

        self.close()