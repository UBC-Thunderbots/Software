from pyqtgraph.Qt import QtWidgets


class RuntimeInstallerDialog(QtWidgets.QDialog):
    """Modal that displays selectable list of runtimes to install"""

    def __init__(
        self,
        parent: QtWidgets.QWidget,
        # TODO (#3559): get list of runtimes from GET request
        runtimes: list[str] = [f"runtime_{i}" for i in range(10)],
    ):
        """Initializes runtime installer modal with list of installable runtimes

        :param parent: the modal's parent
        :param runtimes: list of runtimes available to be installed
        """
        super().__init__(parent)

        self.setWindowTitle("Install runtimes")
        self.setModal(True)
        self.setMinimumWidth(400)

        install_button = QtWidgets.QPushButton("Install")
        install_button.clicked.connect(self.__install_selected_runtimes)

        runtime_select_list = QtWidgets.QListWidget()
        runtime_select_list.setSelectionMode(
            QtWidgets.QAbstractItemView.SelectionMode.MultiSelection
        )
        runtime_select_list.setFixedHeight(200)
        runtime_select_list.addItems(runtimes)

        self.runtime_select_list = runtime_select_list

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(runtime_select_list)
        layout.addWidget(install_button)

    def __install_selected_runtimes(self) -> None:
        """Installs all runtimes that are currently selected"""

        selected_items = self.runtime_select_list.selectedItems()
        selected_runtimes = [item.text() for item in selected_items]

        # TODO (#3559): actually install the list of runtimes
        print("Installing:", selected_runtimes)

        self.close()
