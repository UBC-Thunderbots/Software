from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtWidgets import *
from pyqtgraph import parametertree
from google.protobuf.json_format import MessageToDict
from thefuzz import fuzz
from proto.import_all_protos import *
from software.thunderscope.common import build_parameter_tree


class ProtoConfigurationWidget(QWidget):

    """Creates a searchable parameter widget that can take any protobuf,
    and convert it into a pyqtgraph ParameterTree. This will allow users
    to modify the values.

    """

    def __init__(
        self, proto_to_configure, on_change_callback, search_filter_threshold=60,
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
        self.proto_to_configure = proto_to_configure

        # Create search query bar
        self.search_query = QLineEdit()
        self.search_query.textChanged.connect(self.__handle_search_query_changed)
        self.search_filter_threshold = search_filter_threshold

        # Create ParameterGroup from Protobuf
        self.param_group = parametertree.Parameter.create(
            name="params",
            type="group",
            children=self.config_proto_to_param_dict(
                self.proto_to_configure, read_only=False, search_term=None
            ),
        )

        # Create ParameterTree
        self.param_tree = parametertree.ParameterTree(showHeader=False)
        self.param_tree.setParameters(self.param_group, showTop=False)
        self.param_group.sigTreeStateChanged.connect(self.__handle_parameter_changed)
        self.param_tree.setAlternatingRowColors(False)

        layout.addWidget(self.search_query)
        layout.addWidget(self.param_tree)

    def __handle_search_query_changed(self, search_term):
        """Given a new search term, reconfigure the parameter tree with parameters
        that match the term.

        NOTE: Messages are not searchable, only fields are searchable/filtered

        :param search_term: The term to filter the parameter tree by
        """

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

            self.on_change_callback(child_name, data, self.proto_to_configure)

    def config_proto_to_param_dict(self, message, search_term=None):
        """Converts a protobuf to a pyqtgraph parameter tree dictionary
        that can loaded directly into a ParameterTree

        https://pyqtgraph.readthedocs.io/en/latest/parametertree/index.html

        :param message: The message to convert to a dictionary
        :param search_term: The search filter

        """

        field_list = build_parameter_tree.config_proto_to_field_list(
            message,
            search_term=search_term,
            search_filter_threshold=self.search_filter_threshold,
        )

        self.build_proto(message)

        return field_list

    def build_proto(self, message, current_attr=None):
        """
        Builds the given message
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
