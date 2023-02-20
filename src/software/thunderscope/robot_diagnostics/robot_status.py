from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph import parametertree
from proto.import_all_protos import *
from software.thunderscope.common import build_parameter_tree


class RobotStatusView(QWidget):
    """
    Class to show a detailed view of the robot's current state

    Displays all the information in the RobotStatus message in a collapsible
    tree format
    """

    def __init__(self):
        """
        Initializes the robot status widget
        Builds the parameter tree with a default primitive
        """

        super(RobotStatusView, self).__init__()

        self.robot_status_visible = True

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.param_group = parametertree.Parameter.create(
            name="RobotStatus",
            type="group",
            children=build_parameter_tree.config_proto_to_field_list(
                RobotStatus(), read_only=True, search_term=None
            ),
        )

        self.param_tree = parametertree.ParameterTree(showHeader=False)
        self.param_tree.setParameters(self.param_group, showTop=False)
        self.param_tree.setAlternatingRowColors(False)

        layout.addWidget(self.param_tree)
        self.setMinimumHeight(self.param_tree.rect().height())

        self.toggle_visibility()

    def update(self, new_message, *path):
        """
        Updates the tree with new values from a new message
        :param new_message: the new message to get values from
        :param path: the path of the current message
                      string args starting from the highest parent field
        :return:
        """

        for descriptor in new_message.DESCRIPTOR.fields:
            key = descriptor.name
            value = getattr(new_message, descriptor.name)

            # if a nested message is found, keep searching for the value
            if descriptor.type == descriptor.TYPE_MESSAGE:
                self.update(value, *path, key)
            else:
                # if value is found, update tree
                child = self.param_group.child(*path, key)

                child.setValue(str(value))

    def toggle_visibility(self):
        """
        Toggles the visibility of this widget
        """
        self.robot_status_visible = not self.robot_status_visible

        if self.robot_status_visible:
            self.show()
        else:
            self.hide()
