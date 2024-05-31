import os
from sys import path
from proto.import_all_protos import *
from software.py_constants import MILLISECONDS_PER_SECOND
import logging
from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import QTimer
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph import parametertree
from google.protobuf.json_format import MessageToDict
from thefuzz import fuzz
from proto.import_all_protos import *
from software.thunderscope.common import proto_parameter_tree_util
from PyQt6.QtWidgets import *


class ProtoConfigurationWidget(QWidget):
    """Creates a searchable parameter widget that can take any protobuf,
    and convert it into a pyqtgraph ParameterTree. This will allow users
    to modify the values.

    """

    GLOBAL_SAVE_PATH = "/tmp/tbotspython/thunderbots_configurations_proto"

    def __init__(
        self,
        on_change_callback,
        is_yellow,
        search_filter_threshold=60,
    ):
        """Create a parameter widget given a protobuf

        NOTE: This class handles the ParameterRangeOptions

        :param proto_to_configure: The protobuf we would like to generate
                                   a parameter tree for. This should be
                                   populated with the default values
        :param on_change_callback: The callback to trigger on change
                                    args: name, updated_value, updated_proto
        :param search_filter_threshold: How close should the search query be?
                        100 is an exact match (not ideal), 0 lets everything through

        """
        QWidget.__init__(self)
        layout = QVBoxLayout()
        self.setLayout(layout)

        self.on_change_callback = on_change_callback

        # Create search query bar
        self.search_query = QLineEdit()
        self.search_query.textChanged.connect(self.__handle_search_query_changed)
        self.search_filter_threshold = search_filter_threshold

        self.is_yellow = is_yellow
        self.path_to_file = ProtoConfigurationWidget.GLOBAL_SAVE_PATH + "/"
        if is_yellow:
            self.path_to_file += f"default_yellow.proto"
        else:
            self.path_to_file += f"default_blue.proto"
        self.update_proto_from_file(self.path_to_file)

        # Create ParameterGroup from Protobuf
        self.param_group = parametertree.Parameter.create(
            name="params",
            type="group",
            children=self.config_proto_to_param_dict(
                self.proto_to_configure, search_term=None
            ),
        )

        # Create ParameterTree
        self.param_tree = parametertree.ParameterTree(showHeader=False)
        self.param_tree.setParameters(self.param_group, showTop=False)
        self.param_group.sigTreeStateChanged.connect(self.__handle_parameter_changed)
        self.param_tree.setAlternatingRowColors(False)

        self.save_button = QPushButton("Save")
        self.load_proto_button = QPushButton("Load Proto")
        self.reset_button = QPushButton("Reset")

        self.reset_button.clicked.connect(self.reset_button_callback)
        self.save_button.clicked.connect(self.save_proto_callback)
        self.load_proto_button.clicked.connect(self.load_proto_with_file_explorer)

        self.save_hbox_bottom = QHBoxLayout()
        self.save_hbox_bottom.addWidget(self.load_proto_button)
        self.save_hbox_bottom.addWidget(self.reset_button)

        self.save_hbox_top = QHBoxLayout()
        self.edit_box = QVBoxLayout()

        self.edit_box.addWidget(QLabel("Enter Filename Below To Save:"))
        self.line_box = QLineEdit()
        self.line_box.textChanged.connect(self.textChanged)

        self.line_box.setText(self.path_to_file)
        self.edit_box.addWidget(self.line_box)

        self.save_hbox_top.addLayout(self.edit_box)
        self.save_hbox_top.addWidget(self.save_button)

        layout.addLayout(self.save_hbox_top)
        layout.addLayout(self.save_hbox_bottom)

        layout.addWidget(QLabel("Edit TBotsConfig"))
        layout.addWidget(self.search_query)
        layout.addWidget(self.param_tree)

        # run self.update_on_initialize asynchronously
        self.timer = QTimer()
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.update_on_initialize)
        self.timer.start(3 * MILLISECONDS_PER_SECOND)

    def textChanged(self, text):
        self.path_to_file = text

    def update_proto_from_file(self, path_to_file: str):
        """
        load the protobuf at path_to_file to the variable self.proto_to_configure

        :param path_to_file:
        """
        if not os.path.isfile(path_to_file):
            self.proto_to_configure = ThunderbotsConfig()
            self.proto_to_configure.sensor_fusion_config.friendly_color_yellow = (
                self.is_yellow
            )

            self.build_proto(self.proto_to_configure)
            self.save_current_config_to_file(self.path_to_file)
            return

        logging.info("Loading protobuf from file: {}".format(path_to_file))
        with open(path_to_file, "rb") as f:
            self.proto_to_configure = ThunderbotsConfig()
            self.proto_to_configure.ParseFromString(f.read())

    def update_on_initialize(self):
        """
        Sending the default configuration protobufs to unix_full_system at startup.
        Also updates the widget at the same time.
        """
        logging.info("I've not updated?")
        self.update_proto_from_file(self.path_to_file)

        logging.info("I've updated?")
        self.on_change_callback(None, None, self.proto_to_configure)

        logging.info("I've not updated?")
        self.update_widget()
        self.timer.stop()

    def save_proto_callback(self):
        """
        this is a callback for a button that save the current protobuf to disk!
        """
        logging.info("writing to file")
        # import pudb ; pudb.set_trace()
        self.save_current_config_to_file(self.path_to_file)
        logging.info("wrote to file")

    def save_current_config_to_file(self, path_to_file):
        """
        this save the self.proto_to_configure to path_to_file

        :param path_to_file: the path to file that we are saving to.
        """
        logging.info("writing to file {}".format(path_to_file))

        path_to_directory = os.path.dirname(path_to_file)

        print(f"I am creating directory: {path_to_directory}")
        os.makedirs(path_to_directory, exist_ok=True)

        with open(path_to_file, "wb") as f:
            f.write(self.proto_to_configure.SerializeToString())

    def load_proto_with_file_explorer(self):
        """
        loading the current protobuf through file explorer
        """
        try:
            path_to_file, _ = QFileDialog.getOpenFileName(
                self, "Open file", "/tmp/tbotspython"
            )
            self.path_to_file = path_to_file
            self.update_proto_from_file(path_to_file)

            self.line_box.setText(self.path_to_file)
            self.update_widget()
        except Exception:
            logging.warning(
                "cannot load configuration from {}. Are you sure it is a configuration proto?".format(
                    path_to_file
                )
            )

    def update_widget(self):
        """
        The following function updates the current parameters tree based on the 
        current protobuf that is being configure (i.e. self.proto_to_configure) widget.

        """
        
        # refreshing widgets after the parameters is called
        logging.info("I am updating widget?")
        self.param_group = parametertree.Parameter.create(
            name="params",
            type="group",
            children=self.config_proto_to_param_dict(self.proto_to_configure),
        )

        self.param_tree.setParameters(self.param_group, showTop=False)
        self.param_group.sigTreeStateChanged.connect(self.__handle_parameter_changed)
        self.param_tree.setAlternatingRowColors(False)

    def reset_button_callback(self):
        logging.info("reset button callback has been clicked")
        self.proto_to_configure = ThunderbotsConfig()
        self.proto_to_configure.sensor_fusion_config.friendly_color_yellow = (
            self.is_yellow
        )

        logging.info("updating thunderbots configurations parameters")
        self.build_proto(self.proto_to_configure)
        self.update_widget()

    def __handle_search_query_changed(self, search_term):
        """Given a new search term, reconfigure the parameter tree with parameters
        that match the term.

        NOTE: Messages are not searchable, only fields are searchable/filtered

        :param search_term: The term to filter the parameter tree by
        """

        logging.info("something has changed?")

        self.param_group = parametertree.Parameter.create(
            name="params",
            type="group",
            children=self.config_proto_to_param_dict(
                self.proto_to_configure, search_term
            ),
        )
        self.param_tree.setParameters(self.param_group, showTop=False)
        self.param_group.sigTreeStateChanged.connect(self.__handle_parameter_changed)

    def __handle_parameter_changed(self, param, changes):
        """Handles the parameter change by triggering the provided callback

        :param param: The paramaeter that changed
        :param changes: The changes

        """
        logging.info("Somethign is changing?")
        logging.info(changes)

        for param, change, data in changes:
            path = self.param_group.childPath(param)

            if path is not None:
                child_name = ".".join(path)
            else:
                child_name = param.name()

            # We need to set the updated value, but its hard to differentiate
            # between strings and enums. So we need to try setting the data
            # as a enum first and then as a string. If both raise, then we
            # raise to the main thread because the value wasn't updated.
            #
            # The other types will work with either
            try:
                exec(f"self.proto_to_configure.{child_name} = {data}")
            except (TypeError, NameError):
                exec(f"self.proto_to_configure.{child_name} = data")

            logging.info(f"child name: {child_name} data: {data}")
            self.on_change_callback(child_name, data, self.proto_to_configure)

    def config_proto_to_param_dict(self, message, search_term=None):
        """Converts a protobuf to a pyqtgraph parameter tree dictionary
        that can loaded directly into a ParameterTree

        Also builds a field from the message

        :param message: The message to convert to a dictionary
        :param search_term: The search filter

        """

        field_list = proto_parameter_tree_util.config_proto_to_field_list(
            message,
            search_term=search_term,
            search_filter_threshold=self.search_filter_threshold,
        )

        self.build_proto(message)

        return field_list

    def build_proto(self, message, current_attr=None):
        """
        Builds the given message to a field
        :param message: the message to build
        :param current_attr: the string to execute to access the current level of fields
        :return:
        """

        if not current_attr:
            current_attr = "self.proto_to_configure"

        for descriptor in message.DESCRIPTOR.fields:

            key = descriptor.name
            value = getattr(message, descriptor.name)

            # Protobuf doesn't set the default values by default, and won't let
            # us serialize the message if all the required fields are not set (even
            # if they have a default). So lets just set the default as the value
            if descriptor.type != descriptor.TYPE_MESSAGE:
                if descriptor.type == descriptor.TYPE_STRING:
                    exec(f"{current_attr}.{key} = '{value}'")
                else:
                    exec(f"{current_attr}.{key} = {value}")
            else:
                self.build_proto(value, f"{current_attr}.{key}")
