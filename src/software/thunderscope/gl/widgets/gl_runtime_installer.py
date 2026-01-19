from pyqtgraph.Qt import QtWidgets


class RuntimeInstallerDialog(QtWidgets.QDialog):
    """Modal that displays dropdown of runtimes to install"""

    def __init__(self, parent):
        """Initializes runtime installer modal

        :param parent: the modal's parent
        """
        super().__init__(parent)

        self.setWindowTitle("Install runtimes")
        self.setModal(True)
        self.setMinimumWidth(400)

        layout = QtWidgets.QVBoxLayout(self)

        install_button = QtWidgets.QPushButton("Install")
        install_button.clicked.connect(lambda: print("install from modal"))
        layout.addWidget(install_button)
