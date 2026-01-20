from pyqtgraph.Qt.QtWidgets import (
    QDialog,
    QWidget,
    QPushButton,
    QListWidget,
    QAbstractItemView,
    QVBoxLayout,
)

from software.thunderscope.binary_context_managers.runtime_manager import (
    runtime_manager_instance,
)


class GLRuntimeInstallerDialog(QDialog):
    """Modal that displays selectable list of runtimes to install"""

    def __init__(self, parent: QWidget):
        """Initializes runtime installer modal, fetching a list of installable
        runtimes and adding to a selectable list

        :param parent: the modal's parent
        """
        super().__init__(parent)

        runtimes = runtime_manager_instance.fetch_remote_runtimes()

        self.setWindowTitle("Install runtimes")
        self.setModal(True)
        self.setMinimumWidth(400)

        install_button = QPushButton("Install")
        install_button.clicked.connect(self.__install_selected_runtimes)

        runtime_select_list = QListWidget()
        runtime_select_list.setSelectionMode(
            QAbstractItemView.SelectionMode.MultiSelection
        )
        runtime_select_list.setFixedHeight(200)
        runtime_select_list.addItems(runtimes)

        self.runtime_select_list = runtime_select_list

        layout = QVBoxLayout(self)
        layout.addWidget(runtime_select_list)
        layout.addWidget(install_button)

    def __install_selected_runtimes(self) -> None:
        """Installs all runtimes that are currently selected"""
        selected_items = self.runtime_select_list.selectedItems()
        selected_runtimes = [item.text() for item in selected_items]

        # TODO (#3559): actually install the list of runtimes
        print("Installing:", selected_runtimes)

        self.close()
