import threading
from typing import Callable

from pyqtgraph.Qt.QtCore import pyqtSignal
from pyqtgraph.Qt.QtWidgets import (
    QLabel,
    QLineEdit,
    QListWidget,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class TestRunnerWidget(QWidget):
    """A widget for selecting and running a gameplay test from within Thunderscope.

    Displays a searchable list of discovered tests and a button to run the
    selected one. The test is run on a background thread (so the Qt event loop
    keeps running) via the ``run_callback`` provided at construction; the
    callback's returned status string is reported back to the UI.
    """

    __run_finished_signal = pyqtSignal(str)
    """Emitted with the run's status string when a background run finishes.

    Used to marshal the result back onto the main (Qt) thread.
    """

    def __init__(
        self,
        tests: list[tuple[str, str]],
        run_callback: Callable[[str], str],
    ) -> None:
        """Create the test runner widget.

        :param tests: list of (display_name, test_path) tuples to choose from
        :param run_callback: called on a background thread with the selected
            test's path; should run the test and return a status string to show
        """
        super().__init__()

        self.tests = dict(tests)
        self.run_callback = run_callback

        self.search_query = QLineEdit()
        self.search_query.setPlaceholderText("Search tests")
        self.search_query.textChanged.connect(self.__handle_search_query_changed)

        self.test_list = QListWidget()
        self.test_list.itemDoubleClicked.connect(lambda _: self.__run_selected_test())

        self.run_button = QPushButton("Run Test")
        self.run_button.clicked.connect(self.__run_selected_test)

        self.status_label = QLabel("Select a test to run")

        layout = QVBoxLayout()
        layout.addWidget(self.search_query)
        layout.addWidget(self.test_list)
        layout.addWidget(self.run_button)
        layout.addWidget(self.status_label)
        self.setLayout(layout)

        self.__run_finished_signal.connect(self.__handle_run_finished)
        self.__populate_test_list(search_term=None)

    def __populate_test_list(self, search_term: str | None) -> None:
        """Refill the list with the tests whose name matches the search term.

        :param search_term: the term to filter by, or None to show all tests
        """
        self.test_list.clear()
        for name in sorted(self.tests):
            if not search_term or search_term.lower() in name.lower():
                self.test_list.addItem(name)

    def __handle_search_query_changed(self, search_term: str) -> None:
        """Re-filter the test list when the search query changes.

        :param search_term: the new search query
        """
        self.__populate_test_list(search_term)

    def __run_selected_test(self) -> None:
        """Run the currently selected test on a background thread.

        Does nothing if no test is selected or a run is already in progress.
        """
        selected = self.test_list.currentItem()
        if selected is None or not self.run_button.isEnabled():
            return

        test_path = self.tests[selected.text()]
        self.run_button.setEnabled(False)
        self.status_label.setText(f"Running {selected.text()}...")

        def run() -> None:
            status = self.run_callback(test_path)
            self.__run_finished_signal.emit(status)

        threading.Thread(target=run, daemon=True).start()

    def __handle_run_finished(self, status: str) -> None:
        """Report a finished run's status and re-enable the run button.

        :param status: the status string returned by the run callback
        """
        self.status_label.setText(status)
        self.run_button.setEnabled(True)
