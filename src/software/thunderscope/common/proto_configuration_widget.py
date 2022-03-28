from pyqtgraph.Qt.QtWidgets import QWidget, QVBoxLayout, QLineEdit
from pyqtgraph import parametertree
from google.protobuf.json_format import MessageToDict
from thefuzz import fuzz


class ProtoConfigurationWidget(QWidget):

    """Creates a searchable parameter widget that can take any protobuf,
    and convert it into a pyqtgraph ParameterTree. This will allow users
    to modify the values.

    """

    def __init__(
        self,
        proto_to_configure,
        on_change_callback,
        convert_all_fields_to_bools=False,
        search_filter_threshold=60,
    ):
        """Create a parameter widget given a protobuf

        NOTE: This class handles the ParameterRangeOptions

        :param proto_to_configure: The protobuf we would like to generate
                                   a parameter tree for.
        :param on_change_callback: The callback to trigger on change
        :param search_filter_threshold: How close should the search query be?
                        100 is an exact match (not ideal), 0 lets everything through
        :param convert_all_fields_to_bools: 
                        Sometimes we just want to check/uncheck the fields
                        of the protobuf (and not actually set an explicit value)

        """
        QWidget.__init__(self)
        layout = QVBoxLayout()
        self.setLayout(layout)

        self.search_query = QLineEdit()
        self.search_query.textChanged.connect(self.search_query_changed)
        self.search_filter_threshold = search_filter_threshold
        self.on_change_callback = on_change_callback
        self.convert_all_fields_to_bools = convert_all_fields_to_bools
        self.proto_to_configure = proto_to_configure

        self.param_dict = self.config_proto_to_param_dict(
            self.proto_to_configure(), self.convert_all_fields_to_bools
        )

        self.param_group = parametertree.Parameter.create(
            name="params", type="group", children=self.param_dict
        )

        self.param_tree = parametertree.ParameterTree(showHeader=False)
        self.param_tree.setParameters(self.param_group, showTop=False)

        # TODO (#2528) Remove the setStyleSheet call. Defaults to white theme
        self.param_tree.setStyleSheet(
            """
            QTreeView {
                background-color: '#000000';
                alternate-background-color: '#000000';
                color: white;
            }
            QLabel {
                color: white;
            }

            QTreeView::item:has-children {
                background-color: black;
                color: white;
            }
        """
        )

        self.param_group.sigTreeStateChanged.connect(self.handle_change)

        layout.addWidget(self.search_query)
        layout.addWidget(self.param_tree)

    def search_query_changed(self, search_term):
        self.param_dict = self.config_proto_to_param_dict(
            self.proto_to_configure(), search_term, self.convert_all_fields_to_bools
        )
        self.param_group = parametertree.Parameter.create(
            name="params", type="group", children=self.param_dict
        )
        self.param_tree.setParameters(self.param_group, showTop=False)

    def handle_change(self, param, changes):
        """ TODO
        """
        for param, change, data in changes:
            path = self.param_group.childPath(param)
            if path is not None:
                child_name = ".".join(path)
            else:
                child_name = param.name()
            self.on_change_callback(child_name, data)

    def config_proto_to_param_dict(
        self, message, search_term=None, convert_all_fields_to_bools=False
    ):
        """Converts a protobuf to a pyqtgraph parameter tree dictionary
        that can loaded directly into a ParameterTree

        https://pyqtgraph.readthedocs.io/en/latest/parametertree/index.html

        :param message: The message to convert to a dictionary
        :param search_term: The search filter
        :param convert_all_fields_to_bools: 
                        Sometimes we just want to check/uncheck the fields
                        of the protobuf (and not actually set an explicit value)
    
        """
        message_dict = {}
        field_list = []

        for descriptor in message.DESCRIPTOR.fields:

            key = descriptor.name
            value = getattr(message, descriptor.name)

            if search_term and descriptor.type != descriptor.TYPE_MESSAGE:
                print(fuzz.partial_ratio(search_term, key), key)
                if fuzz.partial_ratio(search_term, key) < self.search_filter_threshold:
                    continue

            if descriptor.type == descriptor.TYPE_MESSAGE:
                field_list.append(
                    {
                        "name": key,
                        "type": "group",
                        "children": self.config_proto_to_param_dict(
                            value, search_term, convert_all_fields_to_bools
                        ),
                    }
                )

            elif convert_all_fields_to_bools or descriptor.type == descriptor.TYPE_BOOL:
                value = False if convert_all_fields_to_bools else value
                field_list.append({"name": key, "type": "bool", "value": value})

            elif descriptor.type == descriptor.TYPE_ENUM:
                options = []

                for enum_desc in descriptor.enum_type.values:
                    options.append(enum_desc.name)

                field_list.append(
                    parametertree.parameterTypes.ListParameter(
                        name=key, default=None, value=None, limits=options + [None]
                    )
                )

            elif descriptor.type == descriptor.TYPE_STRING:
                pass
                # field_list.append(
                    # {
                        # "name": key,
                        # "type": "text",
                        # "value": value,
                        # "default": value,
                    # }
                # )


            elif descriptor.type == descriptor.TYPE_DOUBLE:

                options = MessageToDict(
                    descriptor.GetOptions(), preserving_proto_field_name=True
                )

                try:
                    min_max = options["[TbotsProto.range]"]
                except KeyError:
                    raise KeyError("{} missing ParameterRangeOptions".format(key))

                field_list.append(
                    {
                        "name": key,
                        "type": "slider",
                        "value": value,
                        "default": value,
                        "limits": (
                            float(min_max["min_double_value"]),
                            float(min_max["max_double_value"]),
                        ),
                        "step": 0.01,
                    }
                )

            elif descriptor.type in [descriptor.TYPE_INT32, descriptor.TYPE_INT64]:

                # Extract the options from the descriptor, and store it
                # in the dictionary.
                options = MessageToDict(
                    descriptor.GetOptions(), preserving_proto_field_name=True
                )

                try:
                    min_max = options["[TbotsProto.range]"]
                except KeyError:
                    raise KeyError("{} missing ParameterRangeOptions".format(key))

                field_list.append(
                    {
                        "name": key,
                        "type": "slider",
                        "value": value,
                        "default": value,
                        "limits": (int(min_max["min_int_value"]), int(min_max["max_int_value"])),
                        "step": 1,
                    }
                )

            else:
                raise NotImplementedError(
                    "Unsupported type {} in parameter config".format(descriptor.type)
                )

        if field_list:
            return field_list

        return message_dict
