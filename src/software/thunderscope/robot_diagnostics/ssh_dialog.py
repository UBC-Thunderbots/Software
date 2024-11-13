from pyqtgraph.Qt.QtWidgets import *
from software.embedded.constants.py_constants import RobotPlatform, NetworkConstants
import software.thunderscope.common.common_widgets as common_widgets


class SSHDialog(QDialog):
    """Dialog prompting the user to enter the IP address/hostname and user password
    of a robot to connect to via SSH.

    The dialog also prompts the user to select the robot's compute module platform
    so that, once connected via SSH, we can flash the robot with the appropriate
    binaries or execute platform-specific tasks.
    """

    def __init__(self, parent: QWidget = None, robot_id: int = 0):
        """Initialize the SSHDialog

        :param parent:   the parent of the dialog
        :param robot_id: the ID of the robot to connect to via SSH
                         (used to suggest the IP address of the robot)
        """
        super().__init__(parent)

        self.robot_id = robot_id

        # The IP address/hostname of the robot to connect to via SSH
        self.ssh_host: str | None = None

        # The user password for the robot
        self.ssh_password: str | None = None

        # The robot's compute module platform
        self.robot_platform: RobotPlatform | None = None

        self.__setup_ui()

    def __setup_ui(self) -> None:
        """Initialize the dialog UI with textboxes for entering the robot
        IP/hostname and password
        """
        self.setWindowTitle("SSH into Robot")

        self.ssh_host_textbox = QLineEdit()

        self.ssh_password_textbox = QLineEdit()

        self.ssh_group_box = QGroupBox("SSH")
        ssh_group_box_layout = QVBoxLayout()
        ssh_group_box_layout.addWidget(QLabel("IP Address or Hostname"))
        ssh_group_box_layout.addWidget(self.ssh_host_textbox)
        ssh_group_box_layout.addWidget(QLabel("Password"))
        ssh_group_box_layout.addWidget(self.ssh_password_textbox)
        self.ssh_group_box.setLayout(ssh_group_box_layout)

        self.robot_platform_button_group = QButtonGroup()
        self.robot_platform_group_box, self.robot_platform_buttons = (
            common_widgets.create_radio(
                ("Jetson Nano", "Raspberry Pi"), self.robot_platform_button_group
            )
        )
        self.robot_platform_group_box.setTitle("Robot Platform")

        def jetson_nano_toggled(checked: bool) -> None:
            if checked:
                self.robot_platform = RobotPlatform.NANO
                ip = NetworkConstants.get_ip_address(self.robot_id, RobotPlatform.NANO)
                self.ssh_host_textbox.setText(str(ip))

        def raspberry_pi_toggled(checked: bool) -> None:
            if checked:
                self.robot_platform = RobotPlatform.PI
                ip = NetworkConstants.get_ip_address(self.robot_id, RobotPlatform.PI)
                self.ssh_host_textbox.setText(str(ip))

        self.robot_platform_buttons[0].toggled.connect(jetson_nano_toggled)
        self.robot_platform_buttons[1].toggled.connect(raspberry_pi_toggled)
        self.robot_platform_buttons[0].setChecked(True)

        self.dialog_button_box = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        self.dialog_button_box.accepted.connect(self.accept)
        self.dialog_button_box.rejected.connect(self.reject)

        self.layout = QVBoxLayout()
        self.layout.addWidget(self.ssh_group_box)
        self.layout.addWidget(self.robot_platform_group_box)
        self.layout.addWidget(self.dialog_button_box)
        self.setLayout(self.layout)

    def accept(self) -> None:
        """Override QDialog.accept"""
        self.ssh_host = self.ssh_host_textbox.text()
        self.ssh_password = self.ssh_password_textbox.text()
        super().accept()
