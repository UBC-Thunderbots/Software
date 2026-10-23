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

    Displays a searchable list of discovered tests and a button to run the selected one.
    """

    __run_finished_signal = pyqtSignal(str)

    def __init__(
        self,
        tests: list[tuple[str, str]],
        run_callback: Callable[[str], str],
        cancel_callback: Callable[[], None],
    ) -> None:
        """Create the test runner widget.

        :param tests: list of (display_name, test_path) tuples to choose from
        :param run_callback: called on a background thread with the selected
            test's path; should run the test and return a status string to show
        :param cancel_callback: called to signal an in-progress test to stop
        """
        super().__init__()

        self.tests = dict(tests)
        self.run_callback = run_callback
        self.cancel_callback = cancel_callback
        self.__run_thread = None

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

        self.__run_finished_signal.connect(self.status_label.setText)
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
        """Run the selected test, cancelling any test already in progress."""
        selected = self.test_list.currentItem()
        if selected is None:
            return

        test_path = self.tests[selected.text()]
        self.status_label.setText(f"Running {selected.text()}...")

        # Signal any in-progress run to stop; the new run waits for it to finish.
        self.cancel_callback()
        previous_thread = self.__run_thread

        def run() -> None:
            if previous_thread is not None:
                previous_thread.join()
            status = self.run_callback(test_path)
            # Only report status if this is still the latest requested run, so a
            # cancelled run does not overwrite the new run's status.
            if threading.current_thread() is self.__run_thread:
                self.__run_finished_signal.emit(status)

        self.__run_thread = threading.Thread(target=run, daemon=True)
        self.__run_thread.start()
