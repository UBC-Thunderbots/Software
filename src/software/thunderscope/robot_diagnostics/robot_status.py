from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph import parametertree
from proto.import_all_protos import *
from software.thunderscope.common import build_parameter_tree


class RobotStatusView(QWidget):

    def __init__(self):

        super(RobotStatusView, self).__init__()

        layout = QVBoxLayout()
        self.setLayout(layout)

        self.param_group = parametertree.Parameter.create(
            name="RobotStatus",
            type="group",
            children=build_parameter_tree.config_proto_to_field_list(
                RobotStatus(
                    robot_id=5,
                    power_status=PowerStatus(
                        breakbeam_tripped=True,
                        battery_voltage=10
                    ),
                    error_code=[]
                ),
                read_only=True,
                search_term=None
            )
        )

        self.param_tree = parametertree.ParameterTree(showHeader=False)
        self.param_tree.setParameters(self.param_group, showTop=False)

        layout.addWidget(self.param_tree)
